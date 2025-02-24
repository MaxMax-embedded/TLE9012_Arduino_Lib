classdef TLE9012_Voltage_Sense < realtime.internal.SourceSampleTime & ...
        coder.ExternalDependency
    %
    % System object template for a source block.
    % 
    % This template includes most, but not all, possible properties,
    % attributes, and methods that you can implement for a System object in
    % Simulink.
    %
    % NOTE: When renaming the class name Source, the file name and
    % constructor name must be updated to use the class name.
    %
    
    % Copyright 2016-2018 The MathWorks, Inc.
    %#codegen
    %#ok<*EMCA>
    
    properties
        % Public, tunable properties.
    end
    
    properties (Nontunable)
        % Public, non-tunable properties.
    end
    
    properties (Access = private)
        % Pre-computed constants.
    end
    
    methods
        % Constructor
        function obj = TLE9012_Voltage_Sense(varargin)
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
    end
    
    methods (Access=protected)
        function setupImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation setup code here
            else
                % Call C-function implementing device initialization
                 %coder.cinclude('C:/Users/max/OneDrive/Dokumente/Arduino/libraries/TLE9012_Arduino_Lib/Matlab_S-Functions/inc/TLE9012_Matlab.h');
                 coder.cinclude('C:/Users/max/OneDrive/Dokumente/Arduino/libraries/TLE9012_Arduino_Lib/TLE9012.h');
                 coder.ceval('init_tle9012');
            end
        end
        
        function cell_voltages = stepImpl(obj)   %#ok<MANU>
            cell_voltages = zeros(12,1,'single')
            if isempty(coder.target)
                % Place simulation output code here
                cell_voltages = zeros(12,1,'single')
            else
                % Call C-function implementing device output
                coder.ceval('get_cell_voltages',coder.wref(cell_voltages));
            end
        end
        
        function releaseImpl(obj) %#ok<MANU>
            if isempty(coder.target)
                % Place simulation termination code here
            else
                % Call C-function implementing device termination
                coder.ceval('tle9012_Terminate');
            end
        end
    end
    
    methods (Access=protected)
        %% Define output properties
        function num = getNumInputsImpl(~)
            num = 0;
        end
        
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        
        function varargout = isOutputFixedSizeImpl(~,~)
            varargout{1} = true;
        end
        
        
        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
        end
        
        function varargout = getOutputSizeImpl(~)
            varargout{1} = [12,1];
        end
        
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'single';
        end
        
        function icon = getIconImpl(~)
            % Define a string as the icon for the System block in Simulink.
            icon = 'TLE9012 Voltage Sense';
        end    
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end
    
    methods (Static)
        function name = getDescriptiveName()
            name = 'Source';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')
                % Update buildInfo
                srcDir = fullfile(fileparts(mfilename('fullpath')),'src'); %#ok<NASGU>
                includeDir = fullfile(fileparts(mfilename('fullpath')),'include')
                libDir = 'C:/Users/max/OneDrive/Dokumente/Arduino/libraries/TLE9012_Arduino_Lib'
                addIncludePaths(buildInfo,includeDir);
                addIncludePaths(buildInfo,libDir);
                % Use the following API's to add include files, sources and
                % linker flags
                %addIncludeFiles(buildInfo,'TLE9012.h',libDir);
                addSourceFiles(buildInfo,'TLE9012.cpp',libDir);
                addIncludeFiles(buildInfo,'TLE9012_Matlab.h',includeDir);
                addSourceFiles(buildInfo,'TLE9012_Matlab.cpp',srcDir);
                %addLinkFlags(buildInfo,{'-lSource'});
                %addLinkObjects(buildInfo,'sourcelib.a',srcDir);
                %addCompileFlags(buildInfo,{'-D_DEBUG=1'});
                %addDefines(buildInfo,'MY_DEFINE_1')
            end
        end
    end
end
