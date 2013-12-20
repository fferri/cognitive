prim_fluent(subprocess_exit_code(proc1)).
initially(subprocess_exit_code(proc1), nil).

proc(control,
    [
        say('start subprocess...'),
        subprocess_open(proc1, ['/home/federico/x','2','5']),
        say('waiting 10 seconds...'),
        repeat(10, sleep(1)),
        say('waiting for termination...'),
        subprocess_wait(proc1),
        pi(exit_code, [
            ?(exit_code=subprocess_exit_code(proc1)),
            say(['terminated with exit code ',exit_code])
        ])
    ]
).

