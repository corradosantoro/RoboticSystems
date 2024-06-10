
fact(0, Res) :- Res is 1.
fact(N, Res) :-
	N > 0,
	N1 is N - 1,
	fact(N1, F1),
	Res is N * F1.

