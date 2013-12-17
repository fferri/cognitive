% declare a fluent with the name of the service to receive its response data:
prim_fluent('/randint').
initially('/randint', nil).

proc(call_srv_and_print_resp(Srv,Type,Arg), [call_service(Srv, Type, Arg), pi(v, [?(v='/randint'), say(['response: ', v])])]).

proc(control,
    [
        say('calling /randint service...'),
        call_srv_and_print_resp('/randint', 'randint_server/RandInt', 'randint_server/RandInt'([])),

        say('calling /randint service again...'),
        call_srv_and_print_resp('/randint', 'randint_server/RandInt', 'randint_server/RandInt'([]))
    ]
).

