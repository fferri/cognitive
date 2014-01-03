% declare a fluent with the name of the service to receive its response data:
prim_fluent('/ask').
initially('/ask', nil).

proc(control,
    [
        say('calling /ask service asynchronously...'),
        call_service_async('/ask', 'mixed_initiative_gui/Question', 'mixed_initiative_gui/Question'([message('Are you happy?'), buttons(['Yes','No'])]), 0),

        while('/ask'=nil, [say(...),sleep(1)]),

        pi(x, [?(x='/ask'), say(['answer is: ',x])])
    ]
).

