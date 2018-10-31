classdef VisualizationInterface < InterpretedSystemBase & matlab.system.mixin.CustomIcon

    properties(Nontunable)
        % serverAddress Visualization server address
        serverAddress = '127.0.0.1';
        % serverPort Visualization server port
        serverPort = 9001;
        % identity Identifying string
        identity = 'Nobody'
    end

    properties(Access = protected)
        visualizationServerSocket;
    end

    methods
        function obj = VisualizationInterface( varargin )
            obj@InterpretedSystemBase( varargin{:} );
            obj.systemName = 'VizInt';
        end
    end

    % interface compliance methods
    methods(Access = protected)
        function icon = getIconImpl( obj )
            icon = ['VisInt: ' obj.identity '\n\nServer: ' obj.serverAddress ':' num2str(obj.serverPort)];
        end
    end

    % These are helpers for debugging, logging, and raising comprehensible
    % errors.
    methods(Access = protected)
        function cleanUpNetwork( obj )
            if ~isempty( obj.visualizationServerSocket )
                obj.visualizationServerSocket.close( );
            end
        end

        function sendData( obj, data )
            obj.visualizationServerSocket.write( data );
            obj.log( 'sent data %s', data );
        end

        function connectToServer( obj )
            try
                obj.visualizationServerSocket = edu.scu.engr.rsl.deca.SocketShim( );
            catch e
                switch e.identifier
                case 'MATLAB:undefinedVarOrClass'
                    obj.throw( 'socketShimNotInPath', '"SocketShim.jar" does not appear to be in MATLAB''s Java path.' );
                otherwise
                    throw( e );
                end
            end

            try
                obj.visualizationServerSocket.connect( obj.serverAddress, obj.serverPort );
            catch e
                obj.throw( 'visualizationConnectionError', 'Could not connect to server at "%s:%d"', obj.serverAddress, obj.serverPort );
            end
        end

        function setupImpl( obj )
            obj.logIdentifier = [' ' obj.identity];
            obj.connectToServer( );
        end

        function result = formatData( obj, data )
            if isempty(data)
                result = '';
            else
                result = strjoin( cellfun( @(e) sprintf('%.5g', e), data, 'UniformOutput', false ), ',' );
%                 obj.identity ',' 
                result = [result sprintf('\n')];
            end
        end

        function stepImpl( obj, data )
            obj.sendData( obj.formatData( num2cell(data) ) );
        end

        function releaseImpl(obj)
            obj.log( 'shutting down' );
            obj.cleanUpNetwork( );
        end
    end
end
