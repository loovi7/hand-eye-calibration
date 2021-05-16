function matlab2opencv(variable, fileName, flag, varClass)
temp = variable.TranslationVectors;
% varClass: the variable class waiting for write:
%           'i': for int
%           'f': for float
% flag    : Write mode
%           'w': for write
%           'a': for append
 
[rows, cols] = size(temp);
 
% Beware of Matlab's linear indexing
 temp = temp';
 
% Write mode as default
if ( ~exist('flag','var') )
    flag = 'w';
end
 
% float as default
if ( ~exist('varClass','var') )
    if isfloat(temp)
        varClass = 'f';
    else isinteger(temp)
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
    fprintf( file,  'TranslationVectors%02d: !!opencv-matrix\n', i+1);
    fprintf( file,  '   rows: %d\n', 3);
    fprintf( file,  '   cols: %d\n', 1);
    fprintf( file, ['   dt: ' varClass '\n']);
    fprintf( file,  '   data: [ ');
    for j = 1:3
        fprintf( file, ['%' varClass], temp(i*cols + j));
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


temp = variable.RotationVectors;
[rows, cols] = size(temp);
temp = temp';
m = 1; 
for i=0:rows-1
    fprintf( file,  'RotationVectors%02d: !!opencv-matrix\n', i+1);
    fprintf( file,  '   rows: %d\n', 3);
    fprintf( file,  '   cols: %d\n', 1);
    fprintf( file, ['   dt: ' varClass '\n']);
    fprintf( file,  '   data: [ ');
    for j = 1:3
        fprintf( file, ['%' varClass], temp(i*cols + j));
        if (j == 3)
            fprintf( file, ']\n');
            break
        end
        fprintf( file, ', ');  
    end
end

temp = variable.RadialDistortion;
    fprintf( file,  'Distortion: !!opencv-matrix\n');
    fprintf( file,  '   rows: %d\n', 4);
    fprintf( file,  '   cols: %d\n', 1);
    fprintf( file, ['   dt: ' varClass '\n']);
    fprintf( file,  '   data: [ ');
   
    fprintf( file, ['%' varClass], temp(1));
    fprintf( file, ', ');  
    fprintf( file, ['%' varClass], temp(2));
    fprintf( file, ', ');  
temp = variable.TangentialDistortion;    
    fprintf( file, ['%' varClass], temp(1));
    fprintf( file, ', ');  
    fprintf( file, ['%' varClass], temp(2));
    fprintf( file, ']\n');
    
temp = variable.IntrinsicMatrix;
[rows, cols] = size(temp);
A = temp;
m = 1; 
    fprintf( file,  'IntrinsicMatrix: !!opencv-matrix\n');
    fprintf( file,  '   rows: %d\n', 3);
    fprintf( file,  '   cols: %d\n', 3);
    fprintf( file, ['   dt: ' varClass '\n']);
    fprintf( file,  '   data: [ ');
for i=0:rows-1
    for j = 1:3
        fprintf( file, ['%' varClass], A(i*cols + j));
        if (j == 3)
            if(i~=rows-1)
                fprintf( file, ',\n        ');
                break
            else
                fprintf( file, ']\n');
                break
            end
        end
        fprintf( file, ', ');  
    end
end
fclose(file);

