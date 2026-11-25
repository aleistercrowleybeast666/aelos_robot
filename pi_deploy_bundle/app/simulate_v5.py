"""Deterministic fake sensor/owner. Never imports ROS, serial or sockets."""
from r3_chain import Simulation


class SimulationStop(RuntimeError):pass


class ScenarioBackend(Simulation):
    session='offline-v5'; epoch=1
    def __init__(self,observations=None,cycles=2,fail_op=None,cancel_after=None,pair_available=True):
        super().__init__(); self.rows=list(observations or []); self.stamp=0
        self.cycles=cycles; self.fail_op=fail_op; self.cancel_after=cancel_after
        self.cancelled=False; self.result_events=[]; self.deliveries=0
        self.pair_available=pair_available
        self.blocks_delivered=0
    def check(self):
        if self.cancelled:raise SimulationStop('explicit simulator cancellation')
        if self.cancel_after is not None and len(self.calls)>=self.cancel_after:
            raise SimulationStop('simulated owner takeover')
    def call(self,op,timeout):
        self.check(); self.calls.append(op)
        if op==self.fail_op:raise RuntimeError('injected uncertain protocol completion: '+op)
    def observe_blocks(self,robot):
        self.check(); self.stamp+=1000000000
        row=self.rows.pop(0) if self.rows else {'status':'target','count':1,'target':{'x':.5,'y':.7,'color':'green','isolated_single':True}}
        return dict({'camera_healthy':True,'stamp_ns':self.stamp,'scope':(self.session,self.epoch),
                     'received_monotonic':robot.clock(),'load':'empty'},**row)
    def mission_pair_ops(self):
        if not self.pair_available:
            return frozenset()
        from outbound_pickup_trial import PAIR_REQUIRED_OPS
        return PAIR_REQUIRED_OPS
    def cycle_boundary(self,robot):
        self.deliveries=robot.deliveries; self.result_events=list(robot.events)
        self.blocks_delivered=robot.blocks_delivered
        if robot.deliveries>=self.cycles:
            self.cancelled=True; raise SimulationStop('explicit simulator stop after demonstration cycles')
    def cancel(self):self.cancelled=True


def run(plan,backend=None):
    import task_runner
    backend=backend or ScenarioBackend()
    try:result=task_runner.execute(plan,backend)
    except SimulationStop as exc:
        result={'status':'simulation_cancelled','command':plan['mode'],'reason':str(exc),
                'deliveries':backend.deliveries,'expected_blocks_delivered':backend.blocks_delivered,
                'events':getattr(exc,'route_events',[]),'native_immediate_stop':False}
    return {'status':'simulation_only','calls':backend.calls,'result':result,
            'body_commands':0,'field_readiness_proven':False,'route_evidence':plan['route_evidence'],'uses_demo_chains':True}
