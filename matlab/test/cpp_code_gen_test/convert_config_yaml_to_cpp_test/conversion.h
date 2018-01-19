length(x) ==> x.sze()
size(x) ==> x.size()
size(x,1) ==> x.n_rows
size(x,2) ==> x.n_cols

for loops
while loops
if, else

sum(.) ==> arma::sum(.)
pow(.) ==> arma::pow(.)
exp(.) ==> arma::exp(.)
norm(.) ==> arma::norm(.)

strcmpi(s1,s2) ==> s1.compare(s2)==0

x=zeros(m, n) ==> x = arma::mat(m,n).zeros()

./ ==> /
.* ==> %


x(m:n) ==> x.subvec(m,n)
x(:,m:n) ==> x.cols(m,n)
x(m:n,:) ==> x.rows(m,n)
x(m:n, t:s) ==> x.submat(m,t,n,s)

x = cell(m,1) ==> x = std::vector<>(m)
x{i} ==> x[i]

'some string' ==> "some string"
