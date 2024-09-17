import real_robot_executioner

if __name__ =="__main__":
    a = 6*[0]
    b = 3*[1.5]
    c = 3*[-1.5]
    d = 3*[1.5]
    e = 3*[-1.5]
    
    a.extend(b)
    a.extend(c)
    a.extend(d)
    a.extend(e)
    print(a)
    real_robot_executioner.set_joint_states(a)