% declare a fluent with the name of the service to receive its response data:
prim_fluent('/mixed_initiative_gui/add').
initially('/mixed_initiative_gui/add', nil).

proc(control,
    [
        say('calling service asynchronously...'),
        call_service_async('/mixed_initiative_gui/add', 'mixed_initiative_gui/Question', 'mixed_initiative_gui/Question'([message('Are you happy?'), buttons(['Yes','No'])]), 0),

        while('/mixed_initiative_gui/add'=nil, [say(...),sleep(1)]),

        pi(x, [
            ?(x='/mixed_initiative_gui/add'),
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

