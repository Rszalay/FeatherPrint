; =============================================================================
; FeatherPrint Installer
; Engine-only replacement for UltiMaker Cura 5.13.x
; =============================================================================

#define MyAppName       "FeatherPrint"
#define MyAppVersion    "0.8.0"
#define MyAppPublisher  "Rick Szalay"
#define MyAppURL        "https://github.com/Rszalay/FeatherPrint"

; The Cura release this engine is built against
#define TargetCuraVersion "5.13"
#define TargetCuraMajor   5
#define TargetCuraMinor   13

; Release build output directory (relative to this .iss file)
#define SourceBuildDir "..\build\Release"

; fdmprinter.def.json (relative to this .iss file)
#define FdmPrinterDef "..\packaging\fdmprinter.def.json"

; =============================================================================
[Setup]
AppId={{A3C1E7F2-5D8B-4A9E-B2F4-8E6D0C3A2B5F}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}/issues
AppUpdatesURL={#MyAppURL}/releases

; Stage to temp — actual install is done in [Code] into the detected Cura dir
DefaultDirName={tmp}\FeatherPrintStaging
DisableDirPage=yes

PrivilegesRequired=admin

OutputDir=..\installer_output
OutputBaseFilename=FeatherPrint-{#MyAppVersion}-Windows-x64-Setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern

LicenseFile=..\LICENSE

MinVersion=10.0

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

; =============================================================================
; Files staged to {tmp}\FeatherPrintStaged — copied to Cura dir in [Code]
; NOTE: Arcus.dll and polyclipping.dll are intentionally excluded.
;       Cura's pyArcus.pyd is compiled against those exact files; replacing
;       them breaks the Cura frontend. Arcus is statically linked into
;       CuraEngine.exe so there is no runtime dependency on Arcus.dll.
; =============================================================================
[Files]
Source: "{#SourceBuildDir}\CuraEngine.exe";             DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion
Source: "{#SourceBuildDir}\cura-formulae-engine.dll";   DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbb12.dll";                  DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbbbind_2_5.dll";            DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbbmalloc.dll";              DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#SourceBuildDir}\tbbmalloc_proxy.dll";        DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion skipifsourcedoesntexist
Source: "{#FdmPrinterDef}";                             DestDir: "{tmp}\FeatherPrintStaged"; Flags: ignoreversion

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
      'FeatherPrint targets Cura {#TargetCuraVersion}.x' + #13#10 +
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
    '  2. Replace CuraEngine.exe with FeatherPrint {#MyAppVersion}' + #13#10 +
    '  3. Copy required runtime DLLs (with .bak backups)' + #13#10 +
    '  4. Deploy modified fdmprinter.def.json (with .bak backup)' + #13#10 + #13#10 +
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
  StagingDir:  String;
  DefSrcDir:   String;
  FileNames:   TStringList;
  i:           Integer;
  Src, Dst:    String;
begin
  if CurStep <> ssPostInstall then Exit;

  StagingDir := ExpandConstant('{tmp}\FeatherPrintStaged\');
  DefSrcDir  := CuraInstallDir + '\share\cura\resources\definitions\';

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

  { Back up and replace runtime DLLs (NOT Arcus.dll or polyclipping.dll) }
  FileNames := TStringList.Create;
  try
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

  { Back up and replace fdmprinter.def.json }
  BackupFile(DefSrcDir + 'fdmprinter.def.json');
  CopyFile(StagingDir + 'fdmprinter.def.json', DefSrcDir + 'fdmprinter.def.json', False);

  { Save install location so the uninstaller can find it }
  SaveStringToFile(ExpandConstant('{app}\install_location.txt'),
                   CuraInstallDir, False);

  MsgBox(
    'FeatherPrint installed successfully!' + #13#10 + #13#10 +
    'Location: ' + CuraInstallDir + #13#10 + #13#10 +
    'NOTE: If you update Cura, the official CuraEngine will overwrite this.' + #13#10 +
    'Run this installer again after a Cura update to restore FeatherPrint.',
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
  DefDir:           String;
  FileNames:        TStringList;
  i:                Integer;
begin
  if CurUninstallStep <> usPostUninstall then Exit;

  InstLocationFile := ExpandConstant('{app}\install_location.txt');
  if not FileExists(InstLocationFile) then Exit;

  LoadStringFromFile(InstLocationFile, SavedDirA);
  SavedDir := Trim(String(SavedDirA));
  DefDir   := SavedDir + '\share\cura\resources\definitions\';

  if not DirExists(SavedDir) then Exit;

  RestoreFile(SavedDir + '\CuraEngine.exe');
  RestoreFile(DefDir + 'fdmprinter.def.json');

  FileNames := TStringList.Create;
  try
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
    'FeatherPrint uninstalled.' + #13#10 +
    'Original Cura files have been restored at:' + #13#10 +
    '  ' + SavedDir,
    mbInformation, MB_OK);
end;
