#include <iostream>
using namespace std;

int main() {
    /* Enter your code here. Read input from STDIN. Print output to STDOUT */
    int N;
    cin >> N;
    
    int A[N];
    for (int i=0; i<=N-1; i++){
        cin >> A[i];
    }
    for (int j=N-1; j>=0; j--){
        cout << A[j] << " ";
    }
    cout << endl;   
    return 0;
}