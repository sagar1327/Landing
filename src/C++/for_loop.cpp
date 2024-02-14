#include <iostream>
#include <cstdio>
using namespace std;

int main() {
    // Complete the code.
    int a,b;
    scanf("%d\n%d",&a,&b);
    string names[9] = {"one","two","three","four","five","six","seven","eight","nine"};
    
    for (int i=a; i <= b; i++){
        if (1 <= i and i <= 9){
            cout << names[i-1] << endl;
        }
        else if (9 <= i and i % 2 == 0){
            cout << "even" << endl;
        }
        else{
            cout << "odd" << endl;
        }
    }
    
    return 0;
}
