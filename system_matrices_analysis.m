%% Check controllability and observability of system matrices

A = [1.0, 0; 
     0, 1.0]; 
fprintf("Eigenvalues of A:\n");
disp(eig(A))

B = [0;
     1]; 

C = [1, 0;
     0, 1]; 

check_controllability_observability(A, B, C);

function check_controllability_observability(A, B, C)
    n = size(A, 1);
    
    %% Check controllability
    Co = ctrb(A, B);
    rank_Co = rank(Co);
    
    if rank_Co == n
        fprintf('The system is controllable (rank = %d).\n', rank_Co);
    else
        fprintf('The system is NOT controllable (rank = %d, expected = %d).\n', rank_Co, n);
    end
    
    %% Check observability
    Ob = obsv(A, C);
    rank_Ob = rank(Ob);
    
    if rank_Ob == n
        fprintf('The system is observable (rank = %d).\n', rank_Ob);
    else
        fprintf('The system is NOT observable (rank = %d, expected = %d).\n', rank_Ob, n);
    end
end
