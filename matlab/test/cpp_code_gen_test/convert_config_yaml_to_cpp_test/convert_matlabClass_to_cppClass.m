% function convert_matlabClass_to_cppClass(matlabClass_filename, cppClass_filename)

clc;
close all;
clear;


matlabClass_filename = 'DMP.m';
cppClass_filename = 'DMP_';

[fid, err_msg] = fopen(matlabClass_filename,'r');
if (fid < 0)
    error(err_msg);
end

str = fscanf(fid, '%c');

%% Class description
[str, namespace] = process_class_description_and_namespace(str);


%% find the class name
[str, className] = process_class_name(str);


%% Properties
[str] = process_properties(str);

return

%% find the constructor
['function\s+\w+\s*=\s*' className '\s*('];
[si, ei] = regexp(str, 'classdef\s+\w+');

className = str(si:ei);
[si, ei] = regexp(className, '\s+\w+');
className = className(si+1:ei);

% str = strrep(str, '%%', '/** \brief');
% str = strrep(str, '%', ' *');
% str = strrep(str, 'classdef', 'class');
% str = strrep(str, 'properties', 'protected:');
% str = strrep(str, 'methods', 'public:');
% str = strrep(str, 'param[out]', 'return');
% str = strrep(str, 'end', '}');

% str = regexprep(str, '\s*<\s*handle\s*\*?\s*', '');


% end

function [str, namespace] = process_class_description_and_namespace(str)

[i1, i2] = regexp(str, '%%.+classdef', 'once');
temp = str(i1:i2);
str = str(i2+1:end);

namespace = [];
% find the namespace
[i1, i2] = regexp(temp, '(\n)[^\n]*namespace[^\n]*(\n)');
if (~isempty(i1))
    namespace = temp(i1+1:i2-1);
    temp = [temp(1:i1) temp(i2:end)];
end
namespace = regexprep(namespace, '.*namespace\s+', '');
namespace = regexprep(namespace, '\s*', '');


temp = regexprep(temp, '%%', '/**');
temp = regexprep(temp, '%', ' *');
[~, i2] = regexp(temp, '\*[^\*\n]*(\n)');
temp = [temp(1:i2(end)-1) sprintf('\n') ' */' sprintf('\n') temp(i2(end)+1:end)];

str = [temp str];

end

function [str, className] = process_class_name(str)

[si, ei] = regexp(str, 'classdef\s+\w+');
className = str(si:ei);
className = regexprep(className, 'classdef\s+', '');
% str = regexprep(str, 'classdef', 'class');

str = regexprep(str, ['classdef\s+' className '\s+<\s+handle\s*%'], ['class ' className]);

end

function [str] = process_properties(str)

i1 = regexp(str, '\n\s*properties\s*\n');
i1 = i1(1);
[~,i2] = regexp(str(i1:end), '\n\s*end\s*\n');
i2 = i2(1)+i1-1;

propertiesStr = str(i1:i2);
str1 = str(1:i1-1);
str2 = str(i2+1:end);

str = [str1 propertiesStr str2];


propertiesStr = regexprep(propertiesStr, 'properties', '');
propertiesStr = regexprep(propertiesStr, 'end', '');


newL = sprintf('\n');
tab = sprintf('\t');

propertiesStr = regexp(propertiesStr, '[^\n]*(\n)', 'match');
propertiesStr = strtrim(propertiesStr);
remove_ind = [];
for i=1:length(propertiesStr)
   if (isempty(propertiesStr{i}))
       remove_ind = [remove_ind i];
   else
       break;
   end
end
propertiesStr(remove_ind) = [];

for i=1:length(propertiesStr)
   s = propertiesStr{i};
   if (isempty(s))
       continue;
   elseif (s(1) == '%')
       s = ['//' s(2:end)];
   else
       varType = regexp(s, '\[type:.*\]', 'match');
       varType = varType{1};
       varType = regexprep(varType, '\[type\s*:\s*', '');
       varType = regexprep(varType, '\s*\]', '');
       
       temp = regexp(s, '[^%]+%?', 'match');
       
       varName = temp{1};
       varName = regexprep(varName, '\s*%*\s*', '');
       
       varComment = temp{2};
       varComment = strtrim(varComment);
       varComment = regexprep(varComment, '\[type:.*\]', '');
       
       s = [varType ' ' varName ';' ' ///< ' varComment];
       
%        varType
%        varName
%        varComment
%        s
%        pause
   end
   propertiesStr{i} = s;
end


properties = propertiesStr;
propertiesStr = [newL 'protected:' newL];

for i=1:length(properties)
    if (isempty(properties{i}))
        propertiesStr = [propertiesStr newL];
    else
        propertiesStr = [propertiesStr tab properties{i} newL];
    end
end
propertiesStr= [propertiesStr newL];

str = [str1 propertiesStr str2];

end
