Matrix3d m;
m << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9;
cout << "Here's the matrix m:" << endl << m << endl;
cout << "m.diagonal().asDiagonal() returns: " << endl << m.diagonal().asDiagonal() << endl;
cout << "m.diagonalView() returns: " << endl << m.diagonalView() << endl;
