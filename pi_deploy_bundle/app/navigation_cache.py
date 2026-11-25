"""Owner-scoped sensor subscriptions. No motion interface and no shared pose state."""
import copy,json,threading,time

class NavigationLease:
    def __init__(self,pool,io,cfg):
        self._pool,self._io,self._key=pool,io,pool.key(cfg)
        self._closed=False;self._poisoned=False;self._after=io.clock();self._after_ns=io.ros_now_ns()
    def _assert_open(self):
        if self._closed or self._poisoned:raise RuntimeError('navigation lease closed or poisoned')
    def require_config(self,cfg):
        self._assert_open()
        if self._key!=self._pool.key(cfg):raise RuntimeError('navigation lease config changed')
        return self
    def __getattr__(self,name):
        self._assert_open()
        return getattr(self._io,name)
    def observe(self,tag_id,after_ns,timeout,check):
        self._assert_open()
        return self._io.observe(tag_id,max(self._after_ns,after_ns) if after_ns else 0,timeout,check)
    def observe_checkpoint_id(self,tag_id,after_ns,timeout,check):
        self._assert_open()
        observe=getattr(self._io,'observe_checkpoint_id',self._io.observe)
        return observe(tag_id,max(self._after_ns,after_ns) if after_ns else 0,timeout,check)
    def magnetic_samples(self,after,timeout,check):
        self._assert_open()
        return self._io.magnetic_samples(max(self._after,after),timeout,check)
    def poison(self,reason):
        # A stuck reader retains only this retired object, never the next generation.
        with self._pool.lock:
            self._poisoned=True
            self._pool.error='retired sensor generation: '+str(reason)
            if self._pool.io is self._io:
                self._pool.io=None;self._pool.config_key=None
            self._io.close()
            # Keep active lease until normal release so a second job cannot overlap.
    def close(self):
        with self._pool.lock:
            if self._closed:return
            self._closed=True
            if self._pool.active is self:self._pool.active=None

class PersistentNavigationCache:
    def __init__(self,factory):
        self.factory=factory;self.lock=threading.RLock();self.io=None;self.config_key=None
        self.active=None;self.error=None;self.closed=False;self.generation=0
    @staticmethod
    def key(cfg):return json.dumps(cfg,sort_keys=True,separators=(',',':'),allow_nan=False)
    def _prepare(self,cfg):
        if self.closed:raise RuntimeError('navigation cache closed')
        key=self.key(cfg)
        if self.io is not None and key==self.config_key:return
        if self.active is not None:raise RuntimeError('navigation cache busy; config change rejected')
        if self.io is not None:self.io.close();self.io=None;self.config_key=None
        try:self.io=self.factory(copy.deepcopy(cfg))
        except Exception as exc:self.error=str(exc);raise
        self.config_key=key;self.error=None;self.generation+=1
    def prewarm(self,cfg):
        with self.lock:self._prepare(cfg)
    def acquire(self,cfg):
        with self.lock:
            if self.active is not None:raise RuntimeError('navigation cache lease already active')
            self._prepare(cfg)
            try:
                self.io.begin_task_observation_scope()
                self.active=NavigationLease(self,self.io,cfg)
            except Exception as exc:self.error=str(exc);raise
            return self.active
    def status(self):
        with self.lock:return dict(subscriptions_prepared=self.io is not None and not self.closed,
            sensor_health_verified=False,lease_active=self.active is not None,error=self.error,generation=self.generation,
            body_commands=0,pose_reference_shared=False)
    def close(self):
        with self.lock:
            self.closed=True
            if self.active is not None:self.active.close()
            if self.io is not None:self.io.close();self.io=None
