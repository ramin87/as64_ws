%% read_mat2 function
%  \brief Reads a 2D matrix from stream 'fid'.
%  \details Reads the elements of the matrix row by row.
%  @param[in] fid: The input stream.
%  @param[in] n_rows: The number of rows.
%  @param[in] n_cols: The number of columns.
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
%  @param[out] m: The 2D matrix
function m = read_mat2(fid, n_rows, n_cols, binary)

    if (nargin < 4), binary = false; end % text format
    
    m = zeros(n_rows,n_cols);
    
    if (binary)    
        for i=1:n_rows
            m(i,:) = fread(fid, [1 n_cols], 'float64');
        end  
    else
        for i=1:n_rows
            m(i,:) = fscanf(fid,'%f', n_cols);
        end
        
    end

end
