#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <memory>
using namespace std;

void Recev()
{
    int i = 0;
    while(true)
    {
        cout<<"Teshe"<<endl;
        i++;
    }
}
void Send()
{
    while(true)
    {
        
    }
}

int main()
{
    std::thread thread_(Recev);
    std::thread t2(Send);
    thread_.detach();

    while(true)
    {
        cout<<"Hello world"<<endl;
    }
        
}