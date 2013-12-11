:- lib(regex).

% for accessing msg/srv terms:
dict_get(D,[],D).
dict_get(D,[K|Ks],X) :-
    D =.. [_Type,Dict],
    member(KVPair,Dict),
    KVPair =.. [K,V],
    dict_get(V,Ks,X).

% Path may be an atom in the form field.subfield.subsubfield
dict_get(D,Path,X) :-
    split('\\.', Path, [], FwDots),
    findall(F, (member(Fstr,FwDots), Fstr\=".", atom_string(F,Fstr)), L),
    dict_get(D,L,X).

