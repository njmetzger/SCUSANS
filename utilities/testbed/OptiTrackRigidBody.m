classdef OptiTrackRigidBody < InterpretedSystemBase

    properties(Logical, Nontunable)
        % unicast NatNet uses unicast
        unicast = false
    end

    properties(Nontunable)
        % dllPath NatNetSDK DLL path
        dllPath = fullfile( 'C:\NatNetSDK','lib','x64','NatNetML.dll' );
        % natNetHostAddress NatNet host IP
        natNetHostAddress = '127.0.0.1'
        % natNetClientAddress NatNet client IP
        natNetClientAddress = '127.0.0.1'
        % coordinateTransformationMatrix Frame transform
        coordinateTransformationMatrix = [1 0 0; 0 0 1; 0 -1 0];
        % rigidBodyNames Rigid body names
        rigidBodyNames = {'Nobody', 'Nobody2'};
        % orientationFormat Orientation format
        orientationFormat = 'Euler angles';
    end

    properties(Hidden, Constant, Transient)
        % This is magic that requires the property name with suffix "Set"
        orientationFormatSet = matlab.system.StringSet( {'Euler angles', 'Quaternions'} );
    end

    % Pre-computed constants
    properties(Access = protected)
        rigidBodyIDMap = containers.Map( 'KeyType', 'int32', 'ValueType', 'any' );
        cachedPoses;
        natNetClient;
    end

    methods(Static, Access = protected)
        % Add a tasteful and helpful blurb to the block header.
        function header = getHeaderImpl( )
            header = matlab.system.display.Header( mfilename( 'class' ), ...
            'Title', 'OptiTrackRigidBody', ...
            'Text', ['A custom Simulink block to retrieve position and '...
            'orientation information from OptiTrack via direct integration '...
            'with the NatNet SDK provided by OptiTrack. The output format '...
            'for each rigid body is [x;y;z;roll;pitch;yaw]. If the attitude '...
            'is output as quaternions, they are in the order [qw;qx;qy;qz].'] );
        end

        function groups = getPropertyGroupsImpl( )
           optiTrackGroup = matlab.system.display.SectionGroup( ...
                'Title','OptiTrack',...
                'PropertyList',{'natNetHostAddress', 'natNetClientAddress', ...
                'dllPath', 'unicast'} );

           dataGroup = matlab.system.display.SectionGroup( ...
                'Title','Data',...
                'PropertyList',{'coordinateTransformationMatrix', ...
                'rigidBodyNames', 'orientationFormat', 'debug'} );

           groups = [dataGroup, optiTrackGroup];
        end
    end

    methods
        function obj = OptiTrackRigidBody( varargin )
            obj@InterpretedSystemBase( varargin{:} );
            obj.systemName = 'OptiTrackRigidBody';
        end
    end

    methods(Access = protected)
        function sz = outputSize( obj )
            switch obj.orientationFormat
                case 'Euler angles'
                    sz = 6;
                case 'Quaternions'
                    sz = 7;
                otherwise
                    obj.throw( 'invalidOrientationFormat', 'I don''t know how you did it, but %s is not a valid setting for orientation format.', obj.orientationFormat );
            end
        end

        % Below follows a somewhat moderate amount of magic to automatically
        % produce the number of outputs as specified robots.
        function outputCount = getNumOutputsImpl( obj )
            outputCount = length( obj.rigidBodyNames );
        end

        function varargout = getOutputSizeImpl( obj )
            outputCount = length( obj.rigidBodyNames );
            varargout = cell( 1, outputCount );
            for idx = 1:outputCount
                varargout{idx} = [obj.outputSize( ), 1];
            end
        end

        function varargout = isOutputFixedSizeImpl( obj )
            outputCount = length( obj.rigidBodyNames );
            varargout = cell( 1, outputCount );
            for idx = 1:outputCount
                varargout{idx} = true;
            end
        end

        function varargout = getOutputDataTypeImpl( obj )
            outputCount = length( obj.rigidBodyNames );
            varargout = cell( 1, outputCount );
            for idx = 1:outputCount
                varargout{idx} = 'double';
            end
        end

        function varargout = isOutputComplexImpl( obj )
            outputCount = length( obj.rigidBodyNames );
            varargout = cell( 1, outputCount );
            for idx = 1:outputCount
                varargout{idx} = false;
            end
        end

        function varargout = getOutputNamesImpl( obj )
            varargout = obj.rigidBodyNames;
        end

        function validatePropertiesImpl(obj)
            rbnError = {'invalidProperties', 'Rigid body names must be a cell array of strings (e.g. {''celeste'', ''wisteria''}).'};
            if ~isa( obj.rigidBodyNames, 'cell' )
                obj.throw( rbnError{:} );
            else
                for idx = 1:length(obj.rigidBodyNames)
                    if ~isa( obj.rigidBodyNames{idx}, 'char' )
                        obj.throw( rbnError{:} );
                    end
                end
            end
        end

        function updateIDMap( obj )
            descriptions = obj.natNetClient.GetDataDescriptions( );
            % This is 0-indexed, according to the sample code.
            for idx = 0:(descriptions.Count-1)
                descriptor = descriptions.Item(idx);
                if descriptor.type == 1 % type 1 is rigid bodies
                    name = char(descriptor.Name);
                    nameMatch = strcmpi( obj.rigidBodyNames, name );
                    localIndex = find( nameMatch, 1 );
                    if ~isempty( localIndex )
                        % Map the OptiTrack id for the body with the given name
                        % to its index in the obj.rigidBodyNames array. That array is what determines the
                        obj.log( 'Matching rigid body "%s" %d -> %d', obj.rigidBodyNames{localIndex}, descriptor.ID, localIndex );
                        obj.rigidBodyIDMap(int32(descriptor.ID)) = localIndex;
                    else
                        obj.log( 'A rigid body with unexpected name "%s" appeared!', name );
                    end
                end
            end
        end

        function result = quaternionToEuler( obj, qw, qx, qy, qz )
            % OptiTrack is configured to broadcast rigid body data using a
            % coordinate frame that has the Y-axis pointing up. The Aerospace
            % standard frame that we use in the lab has the Z-axis pointing
            % down, which means the two frames are only offset by a +90-degree
            % rotation around the X-axis.

            % Reminder: roll is rotation about X-axis, pitch is rotation about
            % Y-axis, and yaw is rotation about Z-axis. The convention used
            % requires the rotations to be applied in the order: yaw, pitch,
            % roll, which ends up being rotations about axes z, y', x'', where '
            % indicates an axis that has been changed by the prior rotations.

            % This code is more or less bodily copied from the website:
            % http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
            % with the following terminology changes: "attitude" -> "yaw",
            % "heading" -> "pitch", and "bank" -> "roll". This may be somewhat
            % confusing, but the convention used by that website is that
            % "heading" is a rotation about the Y-axis and "attitude" is a
            % rotation about the Z-axis.

            % Unlike previous iterations of this code, all implicit coordinate
            % frame transformations have been stripped out.

            discriminant = qx*qy + qz*qw;
            if discriminant > 0.49999
                roll = 0;
                pitch = 2*atan2( qx, qw );
                yaw = pi/2;
            elseif discriminant < -0.49999
                roll = 0;
                pitch = -2*atan2( qx, qw );
                yaw = -pi/2;
            else
                sqx = qx^2;
                sqy = qy^2;
                sqz = qz^2;

                roll = atan2( 2*(qx*qw - qy*qz), 1 - 2*(sqx + sqz) );
                pitch = atan2( 2*(qy*qw - qx*qz), 1 - 2*(sqy + sqz) );
                yaw = asin( 2*discriminant );
            end
            result = [roll; pitch; yaw];
        end

        function initializePoseCache( obj )
            obj.cachedPoses = cell( 1, length( obj.rigidBodyNames ) );
            for idx = 1: length(obj.cachedPoses)
                obj.cachedPoses{idx} = zeros( obj.outputSize( ), 1 );
            end
        end

        function setupImpl(obj)
            % Add NatNet .NET assembly so that Matlab can access its methods,
            % delegates, etc. Note: The NatNetML.DLL assembly depends on
            % NatNet.dll, so make sure they are both in the same folder and/or
            % path if you move them.
            try
                NET.addAssembly( obj.dllPath );
            catch e
                if isa( e, 'NET.NetException' )
                    if isa( e.ExceptionObject, 'System.IO.FileNotFoundException' )
                        obj.throw( 'natNetLoadError', 'The provided NatNet library ("%s") could not be found.', obj.dllPath );
                    elseif isa( e.ExceptionObject, 'System.BadImageFormatException' )
                        obj.throw( 'natNetLoadError', 'The provided NatNet library ("%s") is not a valid library', obj.dllPath );
                    end
                else
                    throw( e )
                end
            end

            obj.log( 'Initializing NatNet client.' );
            obj.natNetClient = NatNetML.NatNetClientML( obj.unicast );
            clientVersion = obj.natNetClient.NatNetVersion( );
            obj.log( 'NatNet client version: %d.%d.%d.%d', clientVersion(1), clientVersion(2), clientVersion(3), clientVersion(4) );

            res = obj.natNetClient.Initialize( obj.natNetClientAddress, obj.natNetHostAddress );

            if res == 0
                obj.log( 'Successfully connected to the NatNet host.' );
            else
                obj.releaseImpl( )
                obj.throw( 'natNetConnectionError', 'Failed to connect to NatNet host. Is it running? Am I trying to connect to the correct address? %s', obj.natNetHostAddress );
            end

            obj.initializePoseCache( );
            obj.updateIDMap( )
        end

        function varargout = stepImpl(obj)
            data = obj.natNetClient.GetLastFrameOfData( );

            % For some mysterious region, the descriptions are 0-indexed but
            % the rigid body data is 1-indexed? I've truly dropped directly into
            % hell without passing go or collecting 200 fake dollars.
            varargout = obj.cachedPoses;
            for bodyID = 1:data.nRigidBodies
                rigidBody = data.RigidBodies(bodyID);
                if obj.rigidBodyIDMap.isKey( rigidBody.ID )
                    outputID = obj.rigidBodyIDMap(rigidBody.ID);
                    varargout{outputID}(1:3) = obj.coordinateTransformationMatrix*double([rigidBody.x; rigidBody.y; rigidBody.z]);
                    switch obj.orientationFormat
                        case 'Euler angles'
                            varargout{outputID}(4:end) = obj.coordinateTransformationMatrix*double(obj.quaternionToEuler( rigidBody.qw, rigidBody.qx, rigidBody.qy, rigidBody.qz ));
                        case 'Quaternions'
                            varargout{outputID}(4:end) = double([rigidBody.qw, rigidBody.qx, rigidBody.qy, rigidBody.qz]);
                    end
                end
            end
            obj.cachedPoses = varargout;
        end


        function releaseImpl( obj )
           obj.natNetClient.Uninitialize( );
           obj.log( 'Uninitialized NatNet client.' );
        end
    end
end
