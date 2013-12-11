% declaring a fluent with the name matching the id of a memory term
% will have the fluent matching the memory term whenever this is updated
prim_fluent(foo).
initially(foo, nil).

proc(control,
    [
        say('control started'),

        if(neg(foo=nil), [
            say('there is already a term \'foo\' in the memory; deleting it...'),
            memory_remove(foo)
        ], []),

        say('waiting for term \'foo\' to change in the memory...'),

        prioritized_interrupts([
            interrupt(foo=nil, wait),
            interrupt(neg(foo=nil), pi(x, [
                ?(x=foo),
                say(['somebody wrote foo=', x, '; deleting it...']),
                sleep(1),
                memory_remove(foo),
                sleep(0.5)
            ]))
        ]),

        say('control is terminating')
    ]
).

