function dym_matlab2opencv(variable, MatrixName, fileName, flag, varClass)
 
% varClass: the variable class waiting for write:
%           'i': for int
%           'f': for float
% flag    : Write mode
%           'w': for write
%           'a': for append
 
[rows, cols] = size(variable);
 
% Beware of Matlab's linear indexing
variable = variable';
 
% Write mode as default
if ( ~exist('flag','var') )
    flag = 'w';
end
 
% float as default
if ( ~exist('varClass','var') )
    if isfloat(variable)
        varClass = 'f';
    else isinteger(variable)
        varClass = 'i';
    end
end
 
if ( ~exist(fileName,'file') || flag == 'w' )
    % New file or write mode specified
    file = fopen( fileName, 'w');
    fprintf( file, '%%YAML:1.0\n');
else
    % Append mode
    file = fopen( fileName, 'a');
end
 
% Write variable header
m = 1; 
% fprintf( file,  'Rotation%02d: !!opencv-matrix\n', m);
% fprintf( file,  '   rows: %d\n', rows);
% fprintf( file,  '   cols: %d\n', cols);
% fprintf( file, ['   dt: ' varClass '\n']);
% fprintf( file,  '   data: [ ');
 
% Write variable data
for i=0:rows-1
    fprintf( file,  MatrixName+'%02d: !!opencv-matrix\n', i+1);
    fprintf( file,  '   rows: %d\n', 3);
    fprintf( file,  '   cols: %d\n', 1);
    fprintf( file, ['   dt: ' varClass '\n']);
    fprintf( file,  '   data: [ ');
    for j = 1:3
        fprintf( file, ['%' varClass], variable(i*cols + j));
        if (j == 3)
            fprintf( file, ']\n');
            break
        end
        fprintf( file, ', ');  
    end
%     if mod(i+1,8) == 0
%         fprintf( file, '\n            ');
%     end
end
 
% fprintf( file, ']\n');
 
fclose(file);

