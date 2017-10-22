%% write_mat2 function
%  \brief Writes a 2D matrix in stream 'fid'.
%  \details Writes first the number of rows and columns and then the elements of the matrix row by row.
%  @param[in] m: The 2D matrix
%  @param[in] n_rows: The number of rows
%  @param[in] n_cols: The number of columns
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
function write_mat2(m, n_rows, n_cols, fid, binary)

    if (nargin < 4), fid = 1; end % write to screen
    if (nargin < 5), binary = false; end % text format

    if (binary)
        for i=1:n_rows
            fwrite(fid, m(i,:), 'float64');
        end 
    else 
        for i=1:n_rows
            fprintf(fid, '%f ', m(i,:));
            fprintf(fid, '\n');
        end
    end

end
