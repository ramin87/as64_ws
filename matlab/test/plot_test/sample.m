clc;
close all;
clear;

inArgs = parseInputArguments('name1', 'value1', 'name2','value2', 'FName1','vA');


inArgs


x = 0:0.01:4;
y = x.^2/10 + 0.1*rand(size(x));

span = 10;
z1 = smooth(y, span, 'moving', 2);
z2 = smooth(y, span, 'sgolay', 2);

figure;
hold on;
plot(x,y,'LineWidth', 1.1);
plot(x,z1,'LineWidth', 1.1);
plot(x,z2,'LineWidth', 1.1);
legend('y','z1','z2')
hold off;



function inArgs = parseInputArguments(varargin)

    % function for validating input arguments
    is_string = @(x) assert( ischar(x), 'Value must be a string.');

    % initialize parser with the names and default values of the input arguments
    inPars = inputParser;
    
    inPars.KeepUnmatched = true;

%     inPars.addRequired('req1', is_string);
%     inPars.addRequired('req2', is_string);
    
%     inPars.addOptional('opt1', '', is_string);
%     inPars.addOptional('opt2', '', is_string);
    
    inPars.addParameter('name1', '', is_string);
    inPars.addParameter('name2', '', is_string);

    % Parse input arguments
    inPars.parse(varargin{:});
    
    inArgs = inPars.Results;


end


