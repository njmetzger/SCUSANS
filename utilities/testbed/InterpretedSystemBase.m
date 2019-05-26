classdef(Abstract) InterpretedSystemBase < matlab.System & matlab.system.mixin.Propagates

    properties(Logical, Nontunable)
        % debug Log debug messages
        debug = true;
    end

    properties(Access = protected)
        systemName = 'InterpretedSystemBase';
        logIdentifier = '';
    end

    methods(Static, Access = protected)
        function simMode = getSimulateUsingImpl( )
            simMode = 'Interpreted execution';
        end
        function flag = showSimulateUsingImpl( )
            flag = false;
        end
    end

    methods(Access = protected)
        function log( obj, str, varargin )
            if obj.debug
                disp( obj.logToString( str, varargin{:} ) );
            end
        end

        function message = logToString( obj, str, varargin )
            message = sprintf( [obj.systemName obj.logIdentifier ': ' str], varargin{:} );
        end

        function throw( obj, id, message, varargin )
            throw( MException( [obj.systemName ':' id], message, varargin{:} ) );
        end
    end
end
