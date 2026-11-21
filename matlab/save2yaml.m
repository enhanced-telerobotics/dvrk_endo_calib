function save2yaml(stereoParams, outputDir, options)
%SAVE2YAML Export Stereo Camera Calibrator results for ROS image_pipeline.
    arguments
        stereoParams (1,1) stereoParameters
        outputDir (1,1) string = string(pwd)
        options.LeftName (1,1) string = "stereo_source_left"
        options.RightName (1,1) string = "stereo_source_right"
    end

    imageSize = stereoParams.CameraParameters1.ImageSize;
    validateattributes(imageSize, {'numeric'}, {'vector', 'numel', 2, 'integer', 'positive'});
    if ~isequal(imageSize, stereoParams.CameraParameters2.ImageSize)
        error('dvrk_endo_calib:ImageSizeMismatch', 'The calibrated images must have the same size.');
    end

    [K1, D1, K2, D2, rotation, translation] = stereoParametersToOpenCV(stereoParams);
    metersPerUnit = toMeters(stereoParams.WorldUnits);
    translation = double(translation(:)) * metersPerUnit;
    baseline = norm(translation);
    if ~isfinite(baseline) || baseline <= 0
        error('dvrk_endo_calib:InvalidBaseline', 'Expected a finite, nonzero stereo baseline.');
    end

    % Rectification geometry depends on calibration and image size, so blank images suffice.
    blank = zeros(imageSize, 'uint8');
    [rectified, ~, ~, P1, P2, R1, R2] = rectifyStereoImages( ...
        blank, blank, stereoParams, 'OutputView', 'valid');

    % The returned P coordinates are relative to the first rectified pixel.
    % Center the MATLAB view in the raw-size canvas used by ROS image_proc.
    offset = floor((imageSize - [size(rectified, 1), size(rectified, 2)]) / 2);
    viewShift = [1, 0, offset(2); 0, 1, offset(1); 0, 0, 1];
    P1 = viewShift * P1;
    P2 = viewShift * P2;
    P1(:, 4) = P1(:, 4) * metersPerUnit;
    P2(:, 4) = P2(:, 4) * metersPerUnit;
    if P2(1, 4) >= 0
        error('dvrk_endo_calib:StereoOrder', ...
              'Expected negative right projection Tx. Check the left/right camera order.');
    end

    T = [rotation, translation; 0, 0, 0, 1];
    if ~isfolder(outputDir)
        [success, message] = mkdir(outputDir);
        if ~success
            error('dvrk_endo_calib:CreateOutput', '%s', message);
        end
    end
    writeCameraInfo(fullfile(outputDir, 'left.yaml'), options.LeftName, imageSize, K1, D1, R1, P1);
    writeCameraInfo(fullfile(outputDir, 'right.yaml'), options.RightName, imageSize, K2, D2, R2, P2);
    writeExtrinsics(fullfile(outputDir, 'stereo_extrinsics.yaml'), T, baseline);
end

function scale = toMeters(units)
    switch lower(strtrim(string(units)))
        case {"millimeters", "millimeter", "mm"}
            scale = 1e-3;
        case {"centimeters", "centimeter", "cm"}
            scale = 1e-2;
        case {"meters", "meter", "m"}
            scale = 1;
        case {"inches", "inch", "in"}
            scale = 0.0254;
        otherwise
            error('dvrk_endo_calib:UnknownWorldUnits', ...
                  'Cannot convert calibration world units "%s" to meters.', units);
    end
end

function writeCameraInfo(path, name, imageSize, K, D, R, P)
    [file, cleanup] = openYaml(path); %#ok<ASGLU>
    fprintf(file, 'image_width: %d\nimage_height: %d\n', imageSize(2), imageSize(1));
    fprintf(file, 'camera_name: ''%s''\n', strrep(char(name), '''', ''''''));
    writeMatrix(file, 'camera_matrix', K);
    fprintf(file, 'distortion_model: plumb_bob\n');
    writeMatrix(file, 'distortion_coefficients', reshape(D, 1, []));
    writeMatrix(file, 'rectification_matrix', R);
    writeMatrix(file, 'projection_matrix', P);
end

function writeExtrinsics(path, T, baseline)
    [file, cleanup] = openYaml(path); %#ok<ASGLU>
    fprintf(file, 'transform_direction: left_to_right\ntranslation_units: meters\n');
    fprintf(file, 'baseline_m: %.17g\n', baseline);
    writeMatrix(file, 'T', T);
end

function [file, cleanup] = openYaml(path)
    [file, message] = fopen(path, 'w');
    if file < 0
        error('dvrk_endo_calib:OpenOutput', 'Unable to write %s. %s', path, message);
    end
    cleanup = onCleanup(@() fclose(file));
end

function writeMatrix(file, key, matrix)
    [rows, cols] = size(matrix);
    values = reshape(matrix.', 1, []);
    fprintf(file, '%s:\n  rows: %d\n  cols: %d\n  data: [%.17g', key, rows, cols, values(1));
    fprintf(file, ', %.17g', values(2:end));
    fprintf(file, ']\n');
end
