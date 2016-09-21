#include <tk_be_common/EmergencyEscape/auxiliary.hpp>

/* Some useful namespaces */
using namespace ecl::linear_algebra;
using namespace std;

/*
* readMatrix:
* Import a matrix from a txt file.
* It needs to run once and need to be extremely efficient.
* Adapted from the following link:
* http://stackoverflow.com/questions/14199798/very-generic-argmax-function-in-c-wanted
* Needs a full path to the txt file to work, which is kind of annoying
*/
//template <typename MyMatType>
MatrixXf readMatrix(const char *filename)
{
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    /* Read numbers from file into buffer */
    ifstream infile;
    infile.open(filename);

    /* Check to see if the file was opened successfully */
    if (!infile)
    	cerr << "Could not open file:" << filename << endl;

    /* Collect the data inside the a stream, do this line by line */
    while (!infile.eof())
    {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(!stream.eof())
            stream >> buff[cols*rows + temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();

    if (rows > 1)
        rows--;
    
    /* Now populate matrix with all the number */
    MatrixXf result(rows,cols);
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            result(i,j) = buff[ cols*i+j ];
        }
    }

    return result;
};

SparseMatrix<float> readSparseMatrix(const char *filename)
{

   int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    /* Read numbers from file into buffer */
    ifstream infile;
    infile.open(filename);

    /* Check to see if the file was opened successfully */
    if (!infile)
        cerr << "Could not open file:" << filename << endl;

    /* Collect the data inside the a stream, do this line by line */
    while (!infile.eof())
    {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(!stream.eof())
            stream >> buff[cols*rows + temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();

    rows--;

    /* Now populate matrix with all the number */
    SparseMatrix<float> result(rows,cols-1);
    MatrixXf dense(rows,cols-1);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols-1; j++) {
                result.insert(i,j) = buff[ cols*i+j ];
        }
    }

    result = dense.sparseView();
    //result.makeCompressed();
    
    return result;
}
/* Just a print function to easily print output data in a vector */
void print(const int &n, ...)
{
    va_list arglist;
    int j;
    double sum = 0;
 
    va_start(arglist, n); /* Requires the last fixed parameter (to get the address) */
    for (j = 0; j < n; j++)
        std::cout << va_arg(arglist, double) << "\t"; /* Increments ap to the next argument. */

    va_end(arglist);

    std::cout << endl;
 
}