% declare a fluent with the name of the service to receive its response data:
prim_fluent('/ask').
initially('/ask', nil).

proc(control,
    [
        say('calling /ask service asynchronously...'),
        call_service_async('/ask', 'mixed_initiative_gui/Question', 'mixed_initiative_gui/Question'([message('Are you happy?'), buttons(['Yes','No'])]), 0),

        while('/ask'=nil, [say(...),sleep(1)]),
        pi(raw, [?(raw='/ask'), say(['raw answer: ',raw])]),

        pi(x, [
            ?(x='/ask'),
            pi(y, [
                ?(dict_get(x,text_input,y)),
                say(['y=',y]),
                if(y='Yes',
                    say('the answer is Yes'),
                    say('the answer is No (or something else)')
                )
            ])
        ])
    ]
).

