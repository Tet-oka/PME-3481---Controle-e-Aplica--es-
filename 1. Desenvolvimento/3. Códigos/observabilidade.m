% Load the state matrix A from the workspace
A = Amatrix;

% Define the observation matrix C
% Replace this with your desired observation matrix
C = [1, 0, 0, 0, 0, 0;
     0, 0, 1, 0, 0, 0];

% Get the dimensions of the state matrix A
[n, ~] = size(A);

% Initialize the observability matrix
Obs = transpose(C);

% Build the observability matrix by iteratively multiplying by A
for i = 2:n
    Obs = [Obs; mtimes(transpose(A.^i), transpose(C))];
end

% Display the observability matrix
disp('Observability matrix:');
disp(Obs);

ob = obsv(A,C);
rank(ob)