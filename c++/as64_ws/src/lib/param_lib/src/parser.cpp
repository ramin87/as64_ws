#include <param_lib/parser.h>

namespace param_
{

bool Parser::valid_key(const std::string& key)
{
    if(par_map.find(key) == par_map.end() )
    {
        std::cout << "SigPack: Parameter "+ key + " not found!" <<std::endl;
        return false;
    }
    return true;
}


std::complex<double> Parser::parse_cx(std::string str)
{
    double re,im;
    char i_ch;
    std::stringstream iss(str);

    // Parse full ...
    if(iss >> re >> im >> i_ch && (i_ch=='i'|| i_ch=='j')) return std::complex<double>(re,im);

    // ... or only imag
    iss.clear();
    iss.seekg(0,iss.beg);
    if(iss >> im >> i_ch && (i_ch=='i'|| i_ch=='j')) return std::complex<double>(0.0,im);

    // .. or only real
    iss.clear();
    iss.seekg(0,iss.beg);
    if(iss >> re) return std::complex<double>(re,0.0);

    // ... otherwise
    throw std::invalid_argument("Could not parse complex number!");
}


Parser::Parser(const std::string& fname)
{
    std::ifstream fh;
    std::string line;
    size_t mark = 0;

    // Clear parameter map
    par_map.clear();

    // Open file
    fh.open(fname.c_str());
    if (!fh)
    {
        throw std::ios_base::failure("Could not find " + fname);
    }
    else
    {
        // Parse
        while (std::getline(fh, line))
        {
            std::string keyS="";
            std::string dataS="";

            // Skip empty lines
            if (line.empty())
                continue;

            // Skip lines with only whitespace
            if(line.find_first_not_of("\t ")==std::string::npos)
                continue;

            // Remove comment
            mark = line.find("%");
            if(mark!=std::string::npos)
                line.erase(mark,line.length());

            // Do we have a '='
            mark = line.find("=");
            if(mark!=std::string::npos)
            {
                // Find key
                keyS = line.substr(line.find_first_not_of("\t "),mark-line.find_first_not_of("\t "));
                keyS = keyS.substr(0,keyS.find_last_not_of("\t ")+1);

                // Find data
                dataS = line.substr(mark+1,line.length());
                dataS = dataS.substr(0,dataS.find_last_not_of("\t ")+1);

                // Do we have a string
                mark = dataS.find("\"");
                if(mark!=std::string::npos)
                {
                    dataS = dataS.substr(mark+1,dataS.length());
                    dataS = dataS.substr(0,dataS.find_last_of("\""));
                }
                // Do we have a vector/matrix
                mark = dataS.find("[");
                if(mark!=std::string::npos)
                {
                    dataS = dataS.substr(mark+1,dataS.length());
                    dataS = dataS.substr(0,dataS.find_last_of("]"));
                }

                // Insert to map
                par_map.insert(std::pair<std::string, std::string>(keyS, dataS));
            }
        }

        // Close file
        fh.close();
    }
}


Parser::~Parser() {}


std::string Parser::getString(const std::string key,const std::string def_val)
{
    if(!valid_key(key))
    {
        std::cout << "Setting default " << def_val <<std::endl;
        return def_val;
    }
    return par_map.find(key)->second;
}


arma::cx_vec Parser::getCxCol(const std::string key,const arma::cx_vec def_val)
{
    if(!valid_key(key))
    {
        std::cout << "Setting default \n"<< def_val  <<std::endl;
        return def_val;
    }

    std::string row,str=par_map.find(key)->second;
    std::istringstream full_str(str);
    int K = static_cast<int>(std::count(str.begin(),str.end(),';')+1);
    arma::cx_vec x(K);
    for(int k=0; k<K; k++)
    {
        std::getline(full_str, row, ';');
        x(k) = parse_cx(row);
    }
    return x;
}


arma::cx_rowvec Parser::getCxRow(const std::string key,const arma::cx_rowvec def_val)
{
    if(!valid_key(key))
    {
        std::cout << "Setting default \n"<< def_val  <<std::endl;
        return def_val;
    }

    std::string col,str=par_map.find(key)->second;
    std::istringstream full_str(str);
    int K = static_cast<int>(std::count(str.begin(),str.end(),',')+1);
    arma::cx_rowvec x(K);
    for(int k=0; k<K; k++)
    {
        std::getline(full_str, col, ',');
        x(k) = parse_cx(col);
    }
    return x;
}


arma::cx_mat Parser::getCxMat(const std::string key,const arma::cx_mat def_val)
{
    if(!valid_key(key))
    {
        std::cout << "Setting default \n"<< def_val  <<std::endl;
        return def_val;
    }
    std::string full_str,row,col;
    std::istringstream iss_full;

    full_str = par_map.find(key)->second;
    int R = static_cast<int>(std::count(full_str.begin(),full_str.end(),';')+1);

    iss_full.str(full_str);
    std::getline(iss_full, row, ';');
    int C = static_cast<int>(std::count(row.begin(),row.end(),',')+1);

    arma::cx_mat x(R,C);

    iss_full.seekg(0,iss_full.beg);
    for(int r=0; r<R; r++)
    {
        std::getline(iss_full, row, ';');
        std::istringstream iss_row(row);
        for(int c=0; c<C; c++)
        {
            std::getline(iss_row, col, ',');
            x(r,c)=parse_cx(col);
        }
    }
    return x;
}

} // end namespace
