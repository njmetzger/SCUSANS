classdef Decabot < InterpretedSystemBase & matlab.system.mixin.CustomIcon & matlab.system.mixin.Nondirect

    properties(Nontunable)
        % robotName Robot name
        robotName = 'Nobody';
        % decabotHelper DecabotHelper/DataTurbine server
        decabotHelper = 'localhost';
        % robotAddress Robot network address (optional)
        robotAddress = '';
    end

    properties(Logical, Nontunable)
        % useDataTurbine Use DataTurbine
        useDataTurbine = true;
        % useDirectSocket Use direct robot TCP connection
        useDirectSocket = true;
    end

    properties(Access = protected)
        rbnbSink;
        rbnbSinkMap;
        rbnbSource;
        rbnbSourceMap;

        robotSocket;
        startTime;
        data = [0; 0; 0; 0];
    end

    methods
        function obj = Decabot( varargin )
            obj@InterpretedSystemBase( varargin{:} );
            obj.systemName = 'Decabot';
        end
    end

    % interface compliance methods
    methods(Access = protected)
        % Normally, on Simulink diagram execution, MATLAB performs a code
        % generation phase to automatically determine types and sizes of
        % outputs. Because this block interfaces with Java, it does not support
        % code generation, which causes this step to fail, unless we implement
        % the following methods that explicitly specify the sizes and types of
        % the outputs. There is no need or even way to specify the input sizes
        % and types, as those are determined by the outputs of the blocks
        % feeding into the inputs.
        function varargout = getOutputSizeImpl( ~ )
            varargout = {1, 1, 1, 1};
        end

        function varargout = isOutputFixedSizeImpl( ~ )
            varargout = {true, true, true, true};
        end

        function varargout = getOutputDataTypeImpl( ~ )
            varargout = {'double', 'double', 'double', 'double'};
        end

        function varargout = isOutputComplexImpl( ~ )
            varargout = {false, false, false, false};
        end

        function inCount = getNumInputsImpl( ~ )
            inCount = 3;
        end

        function outCount = getNumOutputsImpl( ~ )
            outCount = 4;
        end

        % By default, Simulink assumes that System objects are direct
        % feedthrough; that is, the outputs of a System block are directly and
        % immediately functions of the inputs to that block. While this
        % assumption in general makes sense for e.g. the common scenario of
        % simple algebraic operations, it does not apply to the Decabot
        % interface. The robot will produce sensor data regardless of the input
        % commands given to it. In order to force Simulink to stop assuming the
        % block is direct feedthrough, the Nondirect protocol can be used. This
        % prevents Simulink from falsely detecting (and failing to solve)
        % algebraic loops where they do not exist, such as in the case that the
        % robots are navigating based on sensor readings.
        function varargout = isInputDirectFeedthroughImpl( ~, ~, ~, ~ )
            varargout = {false, false, false, false};
        end

        function icon = getIconImpl( obj )
            icon = [obj.robotName '\n\nDH: ' obj.decabotHelper ];
        end

        function validatePropertiesImpl(obj)
            if ~(obj.useDataTurbine || obj.useDirectSocket)
                obj.throw( 'invalidProperties', 'Either DataTurbine or the direct TCP connection must be enabled' );
            end
        end
    end


    % These are helpers for debugging, logging, and raising comprehensible
    % errors.
    methods(Access = protected)
        % This function to clear name hogging clients off of the server is
        % ported wholesale from the code included in the ZombieKiller class of
        % the decabot DataTurbine exporter. This has less error checking than
        % that code and is probably less robust, but it appears to work.
        function claimName( obj, name )
            if contains( name, '/' )
                obj.throw( 'badRobotName', 'Robot name parameter may not contain the character "/".' );
            end
            server = com.rbnb.api.Server.newServerHandle( [], obj.decabotHelper );
            controller = server.createController( [name '-grabber'] );
            controller.start( );

            fullName = server.getFullName( ).concat( '/' ).concat( name );

            toFind = com.rbnb.api.Rmap.createFromName( fullName );
            leaf = controller.getRegistered( toFind );
            while leaf.getNchildren( ) > 0
                leaf = leaf.getChildAt( 0 );
            end

            if isa( leaf, 'com.rbnb.api.Client' )
                controller.stop( leaf );
                obj.log( 'DataTurbine name grabbed.' );
            end

            controller.stop( );
        end

        function checkNameUniqueness( obj )
            % This is a mechanism that is used to ensure that no two Decabot
            % blocks in a simulation have the same setting for their robotName
            % property. If two block were to have the same name, cryptic
            % DataTurbine-related errors would occur, as the second one to
            % initialize would automatically disconnect the first.

            % Every single Decabot block in a simulation runs this routine, even
            % though it only needs to be run once. It would be possible to add
            % a property to prevent the redundant checks from occurring, but
            % that has been deemed unnecessarily complex and the amount of saved
            % computation time is likely entirely negligible.
            obj.log( 'running name check' );

            root = bdroot( gcs( ) );

            systemList = find_system( root, 'LookUnderMasks', 'all',...
                'BlockType', 'MATLABSystem', 'System', obj.systemName,...
                'robotName', obj.robotName );

            if length( systemList ) > 1
                systemList = cellfun( @(sysName) {['''' sysName '''']}, systemList );
                obj.throw( 'duplicateRobotName', 'Multiple %s blocks specify the same robotName: %s', obj.systemName, strjoin( systemList, ', ' ) );
            end
        end

        function cleanUpNetwork( obj )
            if obj.useDataTurbine
                obj.rbnbSource.CloseRBNBConnection( );
                obj.rbnbSink.CloseRBNBConnection( );
            end
            if obj.useDirectSocket
                if ~isempty( obj.robotSocket )
                    obj.robotSocket.close( );
                end
            end
        end

        function result = now( obj )
            if isempty( obj.startTime )
                obj.startTime = tic( );
            end
            result = toc( obj.startTime );
        end

        function [new, messages] = fetchDataTCP( obj )
            dump = obj.robotSocket.read( );
            new = dump.length( ) > 0;
            messages = cell( 0 );
            if new
                messages = strsplit( char(dump), '\n' );
            end
        end

        function [new, messages] = fetchDataDataTurbine( obj )
            getmap = obj.rbnbSink.Fetch( 0 );
            % Determine whether or not we got a data packet. If there was
            % no data to be fetched, we just use whatever data was cached
            % in the last iteration.
            new = getmap.NumberOfChannels > 0;
            messages = cell( 0 );
            if new
                messageArray = getmap.GetDataAsString( 0 );
                messageCount = length( messageArray );
                messages = cell( 1, messageCount );
                for idx = 1:messageCount
                    messages{idx} = char( messageArray(idx) );
                end
            end
        end

        function [new, messages] = fetchData( obj )
            if obj.useDirectSocket
                [new, messages] = obj.fetchDataTCP( );
            else
                [new, messages] = obj.fetchDataDataTurbine( );
            end
        end

        function [results] = parseData( obj, messages )
            results = containers.Map( );
            for idx = 1:length( messages )
                dataString = messages{idx};
                obj.log( dataString );
                separators = find( dataString == ';' );
                if isempty( separators )
                    continue
                end
                if length(separators) < 2
                    separators(2) = length(dataString);
                end
                prefix = dataString(1:separators(1)-1);
                timestamp = dataString(separators(1)+1:separators(2)-1);
                % str2num will automatically parse ; separated numbers into a
                % column vector.
                results(prefix) = str2num( dataString(separators(2)+1:end) );
            end
        end

        function sendCommand( obj, command )
            if obj.useDataTurbine
                obj.rbnbSinkMap.PutDataAsString( 0, command );
                obj.rbnbSource.Flush( obj.rbnbSinkMap, true );
            end

            if obj.useDirectSocket
                obj.robotSocket.write( command );
            end

            obj.log( 'sent command %s', command );
        end

        function armRobot( obj, arm )
            obj.sendCommand( obj.armCommand( arm ) );
        end

        function command = formatCommand( obj, namespace, varargin )
            if isempty( varargin )
                body = '';
            else
                body = [';' strjoin( cellfun( @(e) sprintf('%.3f', e), varargin, 'UniformOutput', false ), ';' )];
            end
            command = sprintf( '%s;%.3f%s\n', namespace, obj.now( ), body );
        end

        function command = armCommand( obj, arm )
            command = obj.formatCommand( 'a', arm );
        end

        function command = driveCommand( obj, varargin )
            command = obj.formatCommand( 'd', varargin{:} );
        end

        function command = synchronizeCommand( obj )
            command = obj.formatCommand( 'z' );
        end

        function waitForArmConfirmation( obj )
            obj.log( 'Waiting for arming confirmation.' );
            start = tic( );
            while toc( start ) < 5
                [new, messages] = obj.fetchData( );
                if new
                    results = obj.parseData( messages );
                    if results.isKey( 'a' ) && all(results('a') == 1)
                        obj.log( 'Armed in %gs.', toc( start ) );
                        return
                    end
                end
            end
            obj.throw( 'armFailure', 'Did not receive arming confirmation message from %s within 5 seconds.', obj.robotName );
        end

        function connectToRobot( obj )
            try
                obj.robotSocket.connect( obj.robotAddress, 57028 );
            catch e
                obj.throw( 'robotConnectionError', 'Could not connect to robot at "%s"', obj.robotAddress );
            end
        end

        function lookUpRobot( obj )
            % connect to DecabotHelper robot nameserver.
            try
                obj.robotSocket.connect( obj.decabotHelper, 45168 );
            catch e
                obj.throw( 'decabotHelperConnectionError', 'Could not connect to "%s" to look up robot address.', obj.decabotHelper )
            end

            obj.robotSocket.write( obj.robotName );
            obj.robotSocket.flush( );

            start = tic( );
            while toc( start ) < 5
                message = obj.robotSocket.read( );
                if message.length( ) > 0
                    obj.robotAddress = char( message );
                    obj.robotSocket.close( );
                    obj.connectToRobot( );
                    return
                end
            end

            obj.throw( 'nameserverResposneTimeout', 'DecabotHelper didn''t reply with the robot address in 5 seconds.' );
        end

        function connectTCP( obj )
            try
                obj.robotSocket = edu.scu.engr.rsl.deca.SocketShim( );
            catch e
                switch e.identifier
                case 'MATLAB:undefinedVarOrClass'
                    obj.throw( 'socketShimNotInPath', '"SocketShim.jar" does not appear to be in MATLAB''s Java path.' );
                otherwise
                    throw( e );
                end
            end

            % if robotAddress has been supplied, try to connect directly.
            if isempty( obj.robotAddress )
                obj.lookUpRobot( )
            else
                obj.connectToRobot( )
            end
        end

        function initializeRbnbClients( obj )
            try
                obj.rbnbSource = com.rbnb.sapi.Source( );
                obj.rbnbSink = com.rbnb.sapi.Sink( );
                obj.rbnbSinkMap = com.rbnb.sapi.ChannelMap( );
                obj.rbnbSourceMap = com.rbnb.sapi.ChannelMap( );
            catch e
                switch e.identifier
                case 'MATLAB:undefinedVarOrClass'
                    obj.throw( 'rbnbNotInJavaPath', '"rbnb.jar" does not appear to be in MATLAB''s Java path.' );
                case 'MATLAB:structRefFromNonStruct'
                    obj.throw( 'comOverwritten', 'the Java namespace "com" seems to have been overwritten by a variable somehow.' );
                otherwise
                    throw( e );
                end
            end

            try
                sinkName = [obj.robotName '-simulinkSink'];
                sourceName = [obj.robotName '-controller'];
                obj.claimName( sourceName );

                obj.rbnbSource.OpenRBNBConnection( obj.decabotHelper, sinkName );
                obj.rbnbSink.OpenRBNBConnection( obj.decabotHelper, sourceName );

                obj.rbnbSinkMap.Add( [obj.robotName '/states'] );
                obj.rbnbSink.Monitor( obj.rbnbSinkMap, 0 );

                obj.rbnbSourceMap.Add( 'controller' );
                obj.rbnbSource.Register( obj.rbnbSourceMap );
                obj.rbnbSourceMap.PutTimeAuto( 'next' );
            catch e
                if isa( e, 'matlab.exception.JavaException' )
                    obj.throw( 'rbnbConnectionError', 'could not connect to DataTurbine server.' );
                else
                    throw( e );
                end
            end
        end

        function setupImpl( obj )
            obj.logIdentifier = [' ' obj.robotName];
            if obj.useDataTurbine
                obj.initializeRbnbClients( );
            end
            if obj.useDirectSocket
                obj.connectTCP( );
            end
            obj.sendCommand( obj.synchronizeCommand( ) );
            obj.armRobot( true );
        end

        function [clear, red, green, blue] = outputImpl( obj, ~, ~, ~ )
            obj.log( 'outputImpl %g', obj.now( ) );
            [new, messages] = obj.fetchData( );
            if new
                results = obj.parseData( messages );
                if results.isKey( 's' ) && length( results('s') ) == 4
                    obj.data = results('s');
                end
            end
            clear = obj.data(1);
            red = obj.data(2);
            green = obj.data(3);
            blue = obj.data(4);
        end

        function updateImpl( obj, x_vel, y_vel, t_vel )
            obj.log( 'updateImpl %g', obj.now( ) );
            obj.sendCommand( obj.driveCommand( x_vel, y_vel, t_vel ) );
        end

        function releaseImpl(obj)
            obj.log( 'shutting down' );
            obj.cleanUpNetwork( );
        end
    end
end
