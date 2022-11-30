from rtde_control import RTDEControlInterface as RTDEControl

rtde_frequency = 500.0
rtde_c = RTDEControl("192.168.0.2", rtde_frequency)

# rtde_c = RTDEControl("192.168.0.2")
#%%
print(rtde_c.getActualToolFlangePose())

v = rtde_c.getActualToolFlangePose()


#%%
rtde_c.disconnect()
