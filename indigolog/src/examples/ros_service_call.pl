% declare a fluent with the name of the service to receive its response data:
prim_fluent('/rosout/get_loggers').
initially('/rosout/get_loggers', nil).

proc(control,
    [
        say('calling /rosout/get_loggers service...'),
        call_service('/rosout/get_loggers', 'roscpp/GetLoggers', 'roscpp/GetLoggers'([])),

        pi(v, [
            ?(v='/rosout/get_loggers'),
            say(['response: ', v])
        ])
    ]
).

