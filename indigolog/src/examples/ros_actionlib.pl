proc(move_base_action(GoalID,Frame,X,Y,Theta),
    action_send('move_base', 'move_base_msgs/MoveBase', GoalID, 'move_base_msgs/MoveBaseGoal'([
        target_pose('geometry_msgs/PoseStamped'([
            header('std_msgs/Header'([
                stamp(time(now)),
                frame_id(Frame)
            ])),
            pose('geometry_msgs/Pose'([
                position('geometry_msgs/Point'([x(X), y(Y), z(0.0)])),
                orientation('geometry_msgs/Quaternion'([x(QX), y(QY), z(QZ), w(QW)]))
            ]))
        ]))
    ]))
) :- yield(quaternion_from_euler([0,0,Theta]),[QX,QY,QZ,QW]).

prim_fluent(action_status(move_base)).
initially(action_status(move_base), nil).

proc(print_status(GoalID), [
    action_status(GoalID), pi(x, [?(x=action_status(GoalID)), say(['Goal "',GoalID,'" has status ',x])])
]).

proc(control,
    [
        say('sending goal...'),
        move_base_action(move_base,'/base_link', 0.3, 0.3, 0.0),
        say('waiting completion...'),
        action_wait(move_base),
        say('completed!'),
        print_status(move_base),

        say('sending goal...'),
        move_base_action(move_base,'/base_link', -0.8, -0.1, 0.0),
        say('sleep 8s...'),
        sleep(8),
        print_status(move_base),
        say('abort goal...'),
        action_abort(move_base),

        print_status(move_base),
        sleep(1),
        print_status(move_base)
    ]
).

