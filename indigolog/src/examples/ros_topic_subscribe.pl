% declaring a fluent with the name of the topic makes automatically update it
% when subscribing to that topic:
prim_fluent('/cmd_vel').
initially('/cmd_vel', nil).

proc(control,
    [
        say('subscribing to /cmd_vel topic...'),
        subscribe('/cmd_vel', 'geometry_msgs/Twist'),

        say('waiting for some message on the /cmd_vel topic...'),
        while('/cmd_vel'=nil, sleep(0.5)),

        pi(msg, [
            ?(msg='/cmd_vel'),
            say(['received a message: ', msg]),
            pi(linx, [?(dict_get(msg, 'linear.x', linx)), say(['linear.x value is: ', linx])])
        ])
    ]
).

