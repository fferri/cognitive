prim_fluent(subprocess_exit_code(proc1)).
initially(subprocess_exit_code(proc1), nil).

indigolog_trace(all).

proc(control,
    [
        say('start subprocess...'),
        subprocess_open(proc1, ['bash','-c','for i in 1 2 3 4 5; do echo $i; sleep 1; done; exit 6']),
        say('waiting 3 seconds...'),
        repeat(6, sleep(0.5)),
        say('waiting for termination...'),
        subprocess_wait(proc1),
        pi(exit_code, [
            ?(exit_code=subprocess_exit_code(proc1)),
            say(['terminated with exit code ',exit_code])
        ]),
        repeat(6, sleep(0.5))
    ]
).

