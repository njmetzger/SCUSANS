classdef LoopPacer < InterpretedSystemBase

    properties
        % loopRate Solver Time Step
        loopRate = 0.1;
    end

    properties(Access = private)
        loopCount = 0;
        loopTimer;
    end

    methods(Static, Access = protected)
        function header = getHeaderImpl( )
            header = matlab.system.display.Header( mfilename( 'class' ), ...
            'Title', 'LoopPacer', ...
            'Text', ['A custom Simulink block to slow down the Simulink loop'...
                ' to attempt to match simulation time with wall clock time.'...
                ' Requires a fixed-step solver to work properly.'] );
        end
    end

    methods
        function obj = LoopPacer( varargin )
            obj@InterpretedSystemBase( varargin{:} );
            obj.systemName = 'LoopPacer';
        end
    end

    methods(Access = protected)
        function stepImpl( obj )
            if isempty( obj.loopTimer )
                obj.loopTimer = tic( );
            end
            pause( obj.loopRate*obj.loopCount - toc( obj.loopTimer ) );
            obj.loopCount = obj.loopCount + 1;
        end
    end
end
