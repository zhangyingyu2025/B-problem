"""G11 DEV: exploit slack in MEC guaranteed-clear certificate.

If the current outer source set is contained in B(c,r), r<20, then every robot
position q in B(c,20-r) is guaranteed within 20 m of the true source. Instead of
always travelling to c, choose the point of this certified disk nearest the robot.
"""
import math
from solver_g10 import G10NoDefer

class G11SafeClear(G10NoDefer):
    clear_margin=2e-6
    def source_task(self,c):
        task=super().source_task(c)
        if task[0]!='guaranteed':return task
        t=self.tracks[c]; center=t.mec[0]; r=t.mec[1]
        slack=max(0.,20.-r-self.clear_margin)
        here=self.port.position; d=math.dist(here,center)
        if d<=slack:return ('guaranteed',c,here)
        if d>0 and slack>0:
            q=(center[0]+(here[0]-center[0])*slack/d,
               center[1]+(here[1]-center[1])*slack/d)
            return ('guaranteed',c,q)
        return task
