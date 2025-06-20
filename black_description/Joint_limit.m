function q_mark_under_limit=Joint_limit(q_mark)
Joint_MAX = [ 52,  18,  90,  0,    104, 14,  73,  23,  90,  0,    103, 27,  80, -0,   3,  180,  5,  128,  180,  97, 128];
Joint_MIN = [-68, -23, -60, -135, -74, -21, -55, -18, -60, -135, -74, -12, -83, -50, -3, -120, -107,0  , -134, -5,	0  ];

[m,~]=size(q_mark);
q_mark_under_limit=max(kron(ones(m,1),Joint_MIN),q_mark);
q_mark_under_limit=min(kron(ones(m,1),Joint_MAX),q_mark_under_limit);