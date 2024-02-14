#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
using namespace std;

int main() {
    /*Initialize the n(number of array in a), q(number of queries), k(number of elements in each array),
    and indexes for outputing the elements.*/
    int n, q, k, idx1, idx2;
    //Initialize the elements of the array.
    int element;
    //Initialize the multi-dimentional vetor a.
    vector<vector<int>> a;
    
    //Store the user input in n and q.
    cin >> n >> q;
    //Resize the vector a into n rows or n arrays.
    a.resize(n);

    for (int i = 0; i < n; i++) {
        //Size of current array
        cin >> k;
        //Resize the current array into k elemets.
        a[i].resize(k);
        //Store the inputs into current array.
        for (int j = 0; j < k; j++) {
            cin >> element;
            a[i][j] = element;
        }
    }
    
    for (int m = 0; m < q; m++){
        //Store the idexes
        cin >> idx1 >> idx2;
        /*If the first index is less the number of arrays and second index 
        is less the size of the indexed array, print the indexed element.*/
        if (idx1 < n && idx2 < a[idx1].size()){
            cout << a[idx1][idx2] << "\n";
        }
        else{
            cout << "Invalid index!!\n";
        }
    }
    

    return 0;
}