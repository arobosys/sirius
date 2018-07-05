#include "../include/navigation_class.hpp"
#include <vector>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <climits>

using namespace std;


int main(int argc, char **argv)
{
    ofstream fout("route1.txt"); // создаём объект класса ofstream для записи и связываем его с файлом cppstudio.txt
    navigation_class navclass;
    int st,fp;
    printf("\n Start Point: ");
    scanf("%d", &st);
    printf(" Finish Point : ");
    scanf("%d", &fp);

    string sSt,sFp;
    sSt = "StartA";
    sFp = "PointG";
    std::vector<std::string> marshrut = navclass.optGraphMarshrut(sSt,sFp);
    for (size_t i = 0; i < marshrut.size(); ++i) {
        std::cout << marshrut[i] << ' ' << "\n";
        fout << marshrut[i] << ' ' << "\n"; // запись строки в файл

    }
    fout.close(); // закрываем файл
    system("python3 snake.py");
    return 0;
}