#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

int main()
{
    // Soluzione desiderata per confronto
    Vector2d solution(-1, -1);

    // Definizione delle matrici A e b per il primo sistema
    Matrix2d A1;
    Vector2d b1;
    A1 << 5.547001962252291e-01, -3.770900990025203e-02,
        8.320502943378437e-01, -9.992887623566787e-01;
    b1 << -5.169911863249772e-01, 1.672384680188350e-01;

    // Risoluzione del primo sistema con PALU decomposition
    Vector2d x1_PALU = A1.partialPivLu().solve(b1);
    // Calcolo dell'errore relativo per PALU decomposition
    double relative_error1_PALU = (x1_PALU - solution).norm() / solution.norm();

    // Risoluzione del primo sistema con QR decomposition
    Vector2d x1_QR = A1.householderQr().solve(b1);
    // Calcolo dell'errore relativo per QR decomposition
    double relative_error1_QR = (x1_QR - solution).norm() / solution.norm();

    // Definizione delle matrici A e b per il secondo sistema
    Matrix2d A2;
    Vector2d b2;
    A2 << 5.547001962252291e-01, -5.540607316466765e-01,
        8.320502943378437e-01, -8.324762492991313e-01;
    b2 << -6.394645785530173e-04, 4.259549612877223e-04;

    // Risoluzione del secondo sistema con PALU decomposition
    Vector2d x2_PALU = A2.partialPivLu().solve(b2);
    // Calcolo dell'errore relativo per PALU decomposition
    double relative_error2_PALU = (x2_PALU - solution).norm() / solution.norm();

    // Risoluzione del secondo sistema con QR decomposition
    Vector2d x2_QR = A2.householderQr().solve(b2);
    // Calcolo dell'errore relativo per QR decomposition
    double relative_error2_QR = (x2_QR - solution).norm() / solution.norm();

    // Definizione delle matrici A e b per il terzo sistema
    Matrix2d A3;
    Vector2d b3;
    A3 << 5.547001962252291e-01, -5.547001955851905e-01,
        8.320502943378437e-01, -8.320502947645361e-01;
    b3 << -6.400391328043042e-10, 4.266924591433963e-10;

    // Risoluzione del terzo sistema con PALU decomposition
    Vector2d x3_PALU = A3.partialPivLu().solve(b3);
    // Calcolo dell'errore relativo per PALU decomposition
    double relative_error3_PALU = (x3_PALU - solution).norm() / solution.norm();

    // Risoluzione del terzo sistema con QR decomposition
    Vector2d x3_QR = A3.householderQr().solve(b3);
    // Calcolo dell'errore relativo per QR decomposition
    double relative_error3_QR = (x3_QR - solution).norm() / solution.norm();

    // Output dei risultati
    cout << "System 1: PALU decomposition solution: " << x1_PALU.transpose() << " - Relative error: " << relative_error1_PALU << endl;
    cout << "System 1: QR decomposition solution: " << x1_QR.transpose() << " - Relative error: " << relative_error1_QR << endl;

    cout << "System 2: PALU decomposition solution: " << x2_PALU.transpose() << " - Relative error: " << relative_error2_PALU << endl;
    cout << "System 2: QR decomposition solution: " << x2_QR.transpose() << " - Relative error: " << relative_error2_QR << endl;

    cout << "System 3: PALU decomposition solution: " << x3_PALU.transpose() << " - Relative error: " << relative_error3_PALU << endl;
    cout << "System 3: QR decomposition solution: " << x3_QR.transpose() << " - Relative error: " << relative_error3_QR << endl;

    return 0;
}
