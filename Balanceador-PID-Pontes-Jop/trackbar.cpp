#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Função callback para a trackbar
void onChange(int pos, void* userdata) {
    int value = pos; // Valor atual da posição do cursor
    // Faça algo com o valor, por exemplo, imprimir na tela
    cout << "Valor selecionado: " << value << endl;
}

int main() {
    // Crie uma janela
    namedWindow("Trackbar", WINDOW_NORMAL);
    
    // Valor inicial da trackbar
    int initialValue = 50;
    // Valor máximo da trackbar
    int maxValue = 100;

    // Cria a trackbar
    createTrackbar("Valor", "Trackbar", &initialValue, maxValue, onChange);

    // Loop principal
    while (true) {
        // Aguarda pressionamento de uma tecla por 100ms
        int key = waitKey(100);

        // Se pressionar a tecla 'q', saia do loop
        if (key == 'q') {
            break;
        }
    }

    // Fecha a janela
    destroyAllWindows();

    return 0;
}
