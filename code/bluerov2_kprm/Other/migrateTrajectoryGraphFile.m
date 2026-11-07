function report = migrateTrajectoryGraphFile(inputFile, outputFile)
%MIGRATETRAJECTORYGRAPHFILE 无需重建 PRM 即可迁移旧 MAT 的边存储。
%
% 若 outputFile 与 inputFile 相同，先写入同目录临时文件，校验成功后再
% 原子替换。旧格式仍可由加载器读取，因此迁移是可选的一次性操作。
if nargin<2 || isempty(outputFile), outputFile=inputFile; end
graph=loadKinodynamicRoadmap(inputFile);
sameFile=strcmpi(char(inputFile),char(outputFile));
if sameFile
    tempFile=[char(outputFile),'.migrating.mat'];
else
    tempFile=char(outputFile);
end
report=saveTrajectoryGraph(tempFile,graph);
if sameFile
    movefile(tempFile,char(outputFile),'f');
end
end
