import os

for ld_smooth in [18.0, 19.0, 20.0]:
    os.system(f"roslaunch plan_manage agile_planner_airsim.launch ld_smooth:={ld_smooth}")