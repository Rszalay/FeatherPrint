; =============================================================================
; CuraFeather Installer
; Engine-only replacement for UltiMaker Cura 5.13.x
; =============================================================================

#define MyAppName       "CuraFeather"
#define MyAppVersion    "5.14.0-alpha.0"
#define MyAppPublisher  "CuraFeather Contributors"
#define MyAppURL        "https://github.com/Rszalay/CuraFeather"

; The Cura release this engine is built against
#define TargetCuraVersion "5.13"
#define TargetCuraMajor   5
#define TargetCuraMinor   13

; Release build output directory (relative to this .iss file)
#define SourceBuildDir "..\build\Release"

; =============================================================================
[Setup]
AppId={{B7F3A2E1-9C4D-4B8F-A1E3-7D5C9F2B1A4E}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}/issues
AppUpdatesURL={#MyAppURL}/releases

; Stage to temp — actual install is done in [Code] into the detected Cura dir
DefaultDirName={tmp}\CuraFeatherStaging
DisableDirPage=yes

PrivilegesRequired=admin

OutputDir=..\installer_output
OutputBaseFilename=CuraFeather-{#MyAppVersion}-Windows-x64-Setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern

LicenseFile=..\LICENSE

MinVersion=10.0

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

; =============================================================================
; Files staged to {tmp}\CuraFeatherStaged — copied to Cura dir in [Code]
; =============================================================================
[Files]
Source: "{#SourceBuildDir}\CuraEngine.exe";             DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion
Source: "{#SourceBuildDir}\Arcus.dll";                  DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\polyclipping.dll";           DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\cura-formulae-engine.dll";   DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbb12.dll";                  DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbbbind_2_5.dll";            DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbbmalloc.dll";              DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbbmalloc_proxy.dll";        DestDir: "{tmp}\CuraFeatherStaged"; Flags: ignoreversion skipifsourcedoesntexist

; =============================================================================
[Code]

var
  CuraInstallDir: String;
  CuraFound:      Boolean;
  DetectedVersion: String;

{ --------------------------------------------------------------------------- }
{ Find the Cura install directory                                              }
{ --------------------------------------------------------------------------- }
function FindCuraInstallDir(): String;
var
  BaseDir: String;
  SR: TFindRec;
  BestMatch: String;
begin
  Result := '';
  BestMatch := '';
  BaseDir := 'C:\Program Files\';

  if FindFirst(BaseDir + 'UltiMaker Cura 5*', SR) then
  begin
    repeat
      if SR.Attributes and FILE_ATTRIBUTE_DIRECTORY <> 0 then
      begin
        if Pos('{#TargetCuraVersion}', SR.Name) > 0 then
          BestMatch := BaseDir + SR.Name   { Prefer exact version match }
        else if BestMatch = '' then
          BestMatch := BaseDir + SR.Name;  { Fall back to any Cura 5.x }
      end;
    until not FindNext(SR);
    FindClose(SR);
  end;

  Result := BestMatch;
end;

{ --------------------------------------------------------------------------- }
{ Check the directory contains the expected Cura files                        }
{ --------------------------------------------------------------------------- }
function ValidateCuraDir(Dir: String): Boolean;
begin
  Result := FileExists(Dir + '\CuraEngine.exe') and
            FileExists(Dir + '\UltiMaker-Cura.exe');
end;

{ --------------------------------------------------------------------------- }
{ Read the Cura version string from the EXE resource                         }
{ --------------------------------------------------------------------------- }
function GetCuraVersion(Dir: String): String;
var
  MS, LS: Cardinal;
  Major, Minor: Cardinal;
begin
  Result := 'unknown';
  if GetVersionNumbers(Dir + '\UltiMaker-Cura.exe', MS, LS) then
  begin
    Major := MS shr 16;
    Minor := MS and $FFFF;
    Result := IntToStr(Major) + '.' + IntToStr(Minor);
  end;
end;

{ --------------------------------------------------------------------------- }
{ Backup a file to <file>.bak — keeps only one backup                        }
{ --------------------------------------------------------------------------- }
procedure BackupFile(FilePath: String);
var
  BackupPath: String;
begin
  BackupPath := FilePath + '.bak';
  if FileExists(FilePath) then
  begin
    if FileExists(BackupPath) then
      DeleteFile(BackupPath);
    RenameFile(FilePath, BackupPath);
  end;
end;

{ --------------------------------------------------------------------------- }
{ Restore a .bak file (called during uninstall)                               }
{ --------------------------------------------------------------------------- }
procedure RestoreFile(FilePath: String);
var
  BackupPath: String;
begin
  BackupPath := FilePath + '.bak';
  if FileExists(BackupPath) then
  begin
    if FileExists(FilePath) then
      DeleteFile(FilePath);
    RenameFile(BackupPath, FilePath);
  end;
end;

{ --------------------------------------------------------------------------- }
{ Installer startup — detect Cura and confirm with user                       }
{ --------------------------------------------------------------------------- }
function InitializeSetup(): Boolean;
var
  Msg: String;
  VersionOk: Boolean;
  MajorVer, MinorVer: Cardinal;
  MS, LS: Cardinal;
begin
  Result := False;

  CuraInstallDir := FindCuraInstallDir();
  CuraFound := (CuraInstallDir <> '') and ValidateCuraDir(CuraInstallDir);

  if not CuraFound then
  begin
    MsgBox(
      'UltiMaker Cura {#TargetCuraVersion}.x could not be found on this machine.' + #13#10 + #13#10 +
      'Please install UltiMaker Cura {#TargetCuraVersion} before running this installer.' + #13#10 +
      'Download it from: https://ultimaker.com/software/ultimaker-cura',
      mbError, MB_OK);
    Exit;
  end;

  DetectedVersion := GetCuraVersion(CuraInstallDir);

  { Check version compatibility }
  VersionOk := True;
  if GetVersionNumbers(CuraInstallDir + '\UltiMaker-Cura.exe', MS, LS) then
  begin
    MajorVer := MS shr 16;
    MinorVer := MS and $FFFF;
    VersionOk := (MajorVer = {#TargetCuraMajor}) and (MinorVer = {#TargetCuraMinor});
  end;

  if not VersionOk then
  begin
    if MsgBox(
      'WARNING: Version mismatch detected.' + #13#10 + #13#10 +
      'CuraFeather targets Cura {#TargetCuraVersion}.x' + #13#10 +
      'Detected Cura version: ' + DetectedVersion + #13#10 + #13#10 +
      'Installing on a different version may cause slicing errors.' + #13#10 +
      'Continue anyway?',
      mbConfirmation, MB_YESNO) = IDNO then
    begin
      Exit;
    end;
  end;

  { Confirm with the user }
  Msg :=
    'Detected Cura installation:' + #13#10 +
    '  ' + CuraInstallDir + #13#10 +
    '  Version: ' + DetectedVersion + #13#10 + #13#10 +
    'This installer will:' + #13#10 +
    '  1. Back up CuraEngine.exe  ->  CuraEngine.exe.bak' + #13#10 +
    '  2. Replace CuraEngine.exe with CuraFeather {#MyAppVersion}' + #13#10 +
    '  3. Copy required runtime DLLs (with .bak backups)' + #13#10 + #13#10 +
    'To restore the original engine, run Uninstall from Add/Remove Programs.' + #13#10 + #13#10 +
    'Continue?';

  if MsgBox(Msg, mbConfirmation, MB_YESNO) = IDNO then
    Exit;

  Result := True;
end;

{ --------------------------------------------------------------------------- }
{ Post-install: move staged files into the Cura directory                     }
{ --------------------------------------------------------------------------- }
procedure CurStepChanged(CurStep: TSetupStep);
var
  StagingDir: String;
  FileNames:  TStringList;
  i:          Integer;
  Src, Dst:   String;
begin
  if CurStep <> ssPostInstall then Exit;

  StagingDir := ExpandConstant('{tmp}\CuraFeatherStaged\');

  { Back up and replace CuraEngine.exe }
  BackupFile(CuraInstallDir + '\CuraEngine.exe');
  if not CopyFile(StagingDir + 'CuraEngine.exe',
                  CuraInstallDir + '\CuraEngine.exe', False) then
  begin
    MsgBox('Failed to copy CuraEngine.exe to:' + #13#10 + CuraInstallDir + #13#10 + #13#10 +
           'Check permissions and try again.',
           mbError, MB_OK);
    Exit;
  end;

  { Back up and replace DLLs }
  FileNames := TStringList.Create;
  try
    FileNames.Add('Arcus.dll');
    FileNames.Add('polyclipping.dll');
    FileNames.Add('cura-formulae-engine.dll');
    FileNames.Add('tbb12.dll');
    FileNames.Add('tbbbind_2_5.dll');
    FileNames.Add('tbbmalloc.dll');
    FileNames.Add('tbbmalloc_proxy.dll');

    for i := 0 to FileNames.Count - 1 do
    begin
      Src := StagingDir + FileNames[i];
      Dst := CuraInstallDir + '\' + FileNames[i];
      if FileExists(Src) then
      begin
        BackupFile(Dst);
        CopyFile(Src, Dst, False);
      end;
    end;
  finally
    FileNames.Free;
  end;

  { Save install location so the uninstaller can find it }
  SaveStringToFile(ExpandConstant('{app}\install_location.txt'),
                   CuraInstallDir, False);

  MsgBox(
    'CuraFeather installed successfully!' + #13#10 + #13#10 +
    'Location: ' + CuraInstallDir + #13#10 + #13#10 +
    'NOTE: If you update Cura, the official CuraEngine will overwrite this.' + #13#10 +
    'Run this installer again after a Cura update to restore CuraFeather.',
    mbInformation, MB_OK);
end;

{ --------------------------------------------------------------------------- }
{ Uninstall: restore all .bak files                                           }
{ --------------------------------------------------------------------------- }
procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);
var
  InstLocationFile: String;
  SavedDir:         String;
  SavedDirA:        AnsiString;
  FileNames:        TStringList;
  i:                Integer;
begin
  if CurUninstallStep <> usPostUninstall then Exit;

  InstLocationFile := ExpandConstant('{app}\install_location.txt');
  if not FileExists(InstLocationFile) then Exit;

  LoadStringFromFile(InstLocationFile, SavedDirA);
  SavedDir := Trim(String(SavedDirA));

  if not DirExists(SavedDir) then Exit;

  RestoreFile(SavedDir + '\CuraEngine.exe');

  FileNames := TStringList.Create;
  try
    FileNames.Add('Arcus.dll');
    FileNames.Add('polyclipping.dll');
    FileNames.Add('cura-formulae-engine.dll');
    FileNames.Add('tbb12.dll');
    FileNames.Add('tbbbind_2_5.dll');
    FileNames.Add('tbbmalloc.dll');
    FileNames.Add('tbbmalloc_proxy.dll');

    for i := 0 to FileNames.Count - 1 do
      RestoreFile(SavedDir + '\' + FileNames[i]);
  finally
    FileNames.Free;
  end;

  MsgBox(
    'CuraFeather uninstalled.' + #13#10 +
    'Original CuraEngine has been restored at:' + #13#10 +
    '  ' + SavedDir,
    mbInformation, MB_OK);
end;
