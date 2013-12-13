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
) :- yield(quaternion_from_euler([0,0,Theta]),[QX,QY,QZ,QW]), QW1 is 666*QW, writeln([quat,[QX,QY,QZ,QW1]]).

proc(control,
    [
        say('control started'),

        say('sending goal...'),
        move_base_action(g,'/base_link', 0.8, 0.8, 0.0),
        say('waiting completion...'),
        action_wait(g),
        say('completed!'),

        action_status,

        say('sending goal...'),
        move_base_action(g2,'/base_link', -0.8, -0.8, 0.0),
        say('sleep 8s...'),
        sleep(8),
        say('abort goal...'),
        action_abort(g2),

        action_status,

        say('control is terminating')
    ]
).

