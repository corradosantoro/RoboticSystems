

% object(X) ---> X is an object
% table(X) ---> X is on the table
% upon(X,Y) ---> X is upon Y

:-
    assert(object(cube)),
    assert(object(cylinder)),
    assert(object(prism)),
    assert(object(ring)),
    assert(table(cube)),
    assert(table(cylinder)),
    assert(table(prism)),
    assert(upon(ring, cylinder)).

% object X is free
free(X) :- object(X), \+upon(_,X).

% put object X upon object Y
put(X, Y) :- free(X), free(Y), table(X), retract(table(X)), assert(upon(Y, X)).
%%put(X, Y) :- free(X), free(Y), upon(X, Z), retract(upon(X, Z)), assert(upon(Y, X)).
