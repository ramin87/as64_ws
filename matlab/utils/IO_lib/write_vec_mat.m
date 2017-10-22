%% write_vec_mat function
%  \brief Writes a vector of 2D matrices in stream 'fid'
%  \details Writes the number of 2D matrices, and thern the number of rows and cols and the elements of each 2D matrix row by row.
%  @param[out] m: cell array where each cell has a 2D matrix
%  @param[in] fid: The output stream (optional, default = 1 for output to screen).
%  @param[in] binary: Flag indicating the format (true for binary, false for text, optional, default = false).
function write_vec_mat(m, fid, binary)

    if (nargin < 2), fid = 1; end % write to screen
    if (nargin < 3), binary = false; end % text format

    n_mat = length(m);
    
    if (binary)  
        fwrite(fid, n_mat, 'int64');
    else 
        fprintf(fid, '%i\n', n_mat);
    end

    for k=1:n_mat
        write_mat(m{k}, fid, binary);
    end
end