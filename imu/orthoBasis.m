ortho1 = A(1,:)';

ortho2 = [ minorOf( A, 2, 1 ) minorOf(A, 2,2 ) minorOf( A, 2, 3 ) ]';

ortho3 = [ minorOf( A, 3, 1 ) minorOf(A, 3,2 ) minorOf( A, 3, 3 ) ]';