#include <iostream>
#include <cmath>
#include "fethernet.h"
#include "control.h"
#include "fuzzy.h"
int main() {
    Fuzzy fuzz;
    forKinematic();
    fuzz.target(0, 0, 8, 0, 0, 0);
    
    return 0;
}
