#include <iostream>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

int main()
{
    Vector2d soluzione(-1, -1);      // definisce la soluzione reale dei sistemi



    Matrix2d A1;                     // definisce le matrici A e b per il primo sistema
    Vector2d b1;
    A1 << 5.547001962252291e-01, -3.770900990025203e-02,
        8.320502943378437e-01, -9.992887623566787e-01;
    b1 << -5.169911863249772e-01, 1.672384680188350e-01;

    Vector2d x1_PALU = A1.partialPivLu().solve(b1);             // risolve il primo sistema con la decomposizione PALU
    double errore_relativo_1_PALU = (x1_PALU - soluzione).norm() / soluzione.norm();       // calcola l'errore relativo per la decomposizione PALU

    Vector2d x1_QR = A1.householderQr().solve(b1);              // risolve il primo sistema con la decomposizione QR
    double errore_relativo_1_QR = (x1_QR - soluzione).norm() / soluzione.norm();           // calcola l'errore relativo per la decomposizione QR



    Matrix2d A2;                     // definisce le matrici A e b per il secondo sistema
    Vector2d b2;
    A2 << 5.547001962252291e-01, -5.540607316466765e-01,
        8.320502943378437e-01, -8.324762492991313e-01;
    b2 << -6.394645785530173e-04, 4.259549612877223e-04;

    Vector2d x2_PALU = A2.partialPivLu().solve(b2);            // risolve il secondo sistema con la decomposizione PALU
    double errore_relativo_2_PALU = (x2_PALU - soluzione).norm() / soluzione.norm();       // calcola l'errore relativo per la decomposizione PALU

    Vector2d x2_QR = A2.householderQr().solve(b2);             // risolve il secondo sistema con la decomposizione QR
    double errore_relativo_2_QR = (x2_QR - soluzione).norm() / soluzione.norm();           // calcola l'errore relativo per la decomposizione QR



    Matrix2d A3;                     // definisce le matrici A e b per il terzo sistema
    Vector2d b3;
    A3 << 5.547001962252291e-01, -5.547001955851905e-01,
        8.320502943378437e-01, -8.320502947645361e-01;
    b3 << -6.400391328043042e-10, 4.266924591433963e-10;

    Vector2d x3_PALU = A3.partialPivLu().solve(b3);           // risolve il terzo sistema con la decomposizione PALU
    double errore_relativo_3_PALU = (x3_PALU - soluzione).norm() / soluzione.norm();      // calcola l'errore relativo per la decomposizione PALU

    Vector2d x3_QR = A3.householderQr().solve(b3);            // risolve il terzo sistema con la decomposizione QR
    double errore_relativo_3_QR = (x3_QR - soluzione).norm() / soluzione.norm();          // calcola l'errore relativo per la decomposizione QR


    // output con soluzione computata ed errore relativo

    cout << "Sistema 1: soluzione con dcomposizione PALU: " << x1_PALU.transpose() << "   ---   Errore relativo: " << errore_relativo_1_PALU << endl;
    cout << "Sistema 1: soluzione con dcomposizione QR: " << x1_QR.transpose() << "   ---   Errore relativo: " << errore_relativo_1_QR << endl;
    cout << endl;
    cout << "Sistema 2: soluzione con dcomposizione PALU: " << x2_PALU.transpose() << "   ---   Errore relativo: " << errore_relativo_2_PALU << endl;
    cout << "Sistema 2: soluzione con dcomposizione QR: " << x2_QR.transpose() << "   ---   Errore relativo: " << errore_relativo_2_QR << endl;
    cout << endl;
    cout << "Sistema 3: soluzione con dcomposizione PALU: " << x3_PALU.transpose() << "   ---   Errore relativo: " << errore_relativo_3_PALU << endl;
    cout << "Sistema 3: soluzione con dcomposizione QR: " << x3_QR.transpose() << "   ---   Errore relativo: " << errore_relativo_3_QR << endl;

    return 0;
}
