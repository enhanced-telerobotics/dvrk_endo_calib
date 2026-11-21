function runDir = open_stereo_calibrator(runDir, squareSizeMM)
%OPEN_STEREO_CALIBRATOR Start a new MATLAB session for a captured stereo run.
    arguments
        runDir (1,1) string = ""
        squareSizeMM (1,1) double = NaN
    end

    if runDir == ""
        root = fullfile(getenv('HOME'), '.config', 'dvrk_endo_calib', 'captures');
        runs = dir(fullfile(root, '*Z'));
        [~, order] = sort(string({runs.name}), 'descend');
        for index = order
            run = runs(index);
            candidate = string(fullfile(run.folder, run.name));
            if run.isdir && hasImages(fullfile(candidate, 'left')) ...
                    && hasImages(fullfile(candidate, 'right'))
                runDir = candidate;
                break
            end
        end
        if runDir == ""
            error('dvrk_endo_calib:NoCaptureRun', 'No stereo captures found in %s.', root);
        end
    end

    leftDir = fullfile(runDir, 'left');
    rightDir = fullfile(runDir, 'right');
    if ~hasImages(leftDir) || ~hasImages(rightDir)
        error('dvrk_endo_calib:MissingStereoImages', ...
              'Expected images in both left/ and right/ under %s.', runDir);
    end

    if isnan(squareSizeMM)
        manifest = fullfile(runDir, 'run.yaml');
        if isfile(manifest)
            % image_collect writes square_size_mm as an indented numeric YAML scalar.
            match = regexp(fileread(manifest), ...
                '(?m)^[ \t]+square_size_mm:[ \t]*([^#\r\n]+)', 'tokens', 'once');
            if ~isempty(match)
                squareSizeMM = str2double(strtrim(match{1}));
            end
        end
        if isnan(squareSizeMM)
            error('dvrk_endo_calib:MissingSquareSize', ...
                  'Pass the square side length in millimeters as the second argument.');
        end
    end
    validateattributes(squareSizeMM, {'double'}, {'real', 'finite', 'positive'});

    stereoCameraCalibrator(leftDir, rightDir, squareSizeMM, 'millimeters');
end

function result = hasImages(folder)
    files = dir(fullfile(folder, '*'));
    [~, ~, extensions] = fileparts(string({files(~[files.isdir]).name}));
    result = any(ismember(lower(extensions), [".jpg", ".jpeg", ".png", ".tif", ".tiff"]));
end
