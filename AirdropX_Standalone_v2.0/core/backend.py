from pathlib import Path
from .simulation import StandaloneSimulation
class StandaloneBackend:
    def __init__(self,app_root:Path): self.sim=StandaloneSimulation(app_root)
    def stop(self): self.sim.stop()
    def run(self,cfg,frame_cb=None,log_cb=None,progress_cb=None): return self.sim.run(cfg,frame_cb,log_cb,progress_cb)
