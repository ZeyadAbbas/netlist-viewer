/////////////////////////////////////////////////////////////////////////////
// Name:        app.cpp
// Purpose:     Spice Viewer application entry point
// Author:      Francesco Montorsi
// Created:     28/05/2010
// Copyright:   (c) Francesco Montorsi
// Licence:     GPL license
/////////////////////////////////////////////////////////////////////////////

// ============================================================================
// declarations
// ============================================================================

// ----------------------------------------------------------------------------
// headers
// ----------------------------------------------------------------------------
 
// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"


// for all others, include the necessary headers (this file is usually all you
// need because it includes almost all "standard" wxWidgets headers)
#ifndef WX_PRECOMP
    #include "wx/wx.h"
#endif

#include <wx/aboutdlg.h>
#include <wx/dcbuffer.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <wx/log.h>
#include <wx/stdpaths.h>
#include <wx/filename.h>

#include "netlist.h"
#include "devices.h"
#include <cstdio>
#include <fstream>
#include <algorithm>

// ----------------------------------------------------------------------------
// constants
// ----------------------------------------------------------------------------

#define SW_VERSION_STR         "0.4"
#define SW_COPYRIGHT_STR       "(C) 2010-2025"
#define HELP_PAGE              "https://github.com/f18m/netlist-viewer/issues"

// file dialog filters
#define FILTER_NETLISTVIEWERSCHEMATIC_FILES \
    "NetlistViewer schematic (*.nvs)|*.nvs"
#define FILTER_SPICENETLIST_FILES \
    "SPICE netlists (*.net;*.cir;*.ckt)|*.net;*.cir;*.ckt"
#define FILTER_ALL_FILES \
    "All files (*.*)|*.*"

#define FILTER_SPICENETLIST \
    FILTER_SPICENETLIST_FILES "|" \
    FILTER_ALL_FILES

#define FILTER_NETLISTVIEWERSCHEMATIC \
    FILTER_NETLISTVIEWERSCHEMATIC_FILES "|" \
    FILTER_ALL_FILES

static const unsigned int DEFAULT_GRID_SIZE = 40;

struct svHeadlessOptions
{
    wxString inputPath;
    wxString outputPath;
    bool showHelp;

    svHeadlessOptions()
        : showHelp(false)
    {
    }
};

class svScopedLogTarget
{
    wxLog* m_previous;

public:
    explicit svScopedLogTarget(wxLog* target)
        : m_previous(wxLog::SetActiveTarget(target))
    {
    }

    ~svScopedLogTarget()
    {
        wxLog::SetActiveTarget(m_previous);
    }
};

static bool HasHeadlessCommandLineArguments(int argc, wxChar** argv)
{
    for (int i=1; i<argc; i++)
    {
        const wxString arg(argv[i]);
        if (arg == wxT("-i") || arg == wxT("--input") ||
            arg == wxT("-o") || arg == wxT("--output") ||
            arg == wxT("-h") || arg == wxT("--help"))
        {
            return true;
        }
    }

    return false;
}

static void PrintCommandLineUsage(FILE* stream, const wxString& programName)
{
    wxFprintf(stream,
        wxT("Netlist Viewer\n")
        wxT("Usage:\n")
        wxT("  %s\n")
        wxT("  %s --input <netlist> --output <image.png>\n")
        wxT("  %s --help\n")
        wxT("\n")
        wxT("Options:\n")
        wxT("  -i, --input <netlist>    SPICE/PSpice netlist to render.\n")
        wxT("  -o, --output <image.png> PNG image to create.\n")
        wxT("  -h, --help               Show this help.\n"),
        programName.c_str(), programName.c_str(), programName.c_str());
}

static bool ParseHeadlessCommandLine(int argc, wxChar** argv,
                                     svHeadlessOptions* options,
                                     wxString* error)
{
    for (int i = 1; i < argc; ++i)
    {
        const wxString arg(argv[i]);

        if (arg == wxT("-h") || arg == wxT("--help"))
        {
            options->showHelp = true;
        }
        else if (arg == wxT("-i") || arg == wxT("--input"))
        {
            if (!options->inputPath.empty())
            {
                *error = wxT("The input path was specified more than once.");
                return false;
            }
            if (++i >= argc)
            {
                *error = wxT("Missing path after --input.");
                return false;
            }
            options->inputPath = argv[i];
        }
        else if (arg == wxT("-o") || arg == wxT("--output"))
        {
            if (!options->outputPath.empty())
            {
                *error = wxT("The output path was specified more than once.");
                return false;
            }
            if (++i >= argc)
            {
                *error = wxT("Missing path after --output.");
                return false;
            }
            options->outputPath = argv[i];
        }
        else
        {
            *error = wxString::Format(wxT("Unknown option: %s"), arg.c_str());
            return false;
        }
    }

    if (options->showHelp)
        return true;

    if (options->inputPath.empty() || options->outputPath.empty())
    {
        *error = wxT("Headless rendering requires both --input and --output.");
        return false;
    }

    wxFileName outputFile(options->outputPath);
    if (outputFile.GetExt().CmpNoCase(wxT("png")) != 0)
    {
        *error = wxT("The output path must use the .png extension.");
        return false;
    }

    return true;
}

static bool RenderNetlistToPng(const wxString& inputPath,
                               const wxString& outputPath,
                               wxString* error)
{
    wxFileName inputFile(inputPath);
    if (!inputFile.FileExists())
    {
        *error = wxString::Format(wxT("Input netlist does not exist: %s"), inputPath.c_str());
        return false;
    }

    svParserSPICE parser;
    svCircuitArray subcktArray;
    if (!parser.load(subcktArray, inputPath.ToStdString()))
    {
        *error = wxString::Format(wxT("Failed to parse netlist: %s"), inputPath.c_str());
        return false;
    }

    if (subcktArray.empty())
    {
        *error = wxString::Format(
            wxT("The netlist did not contain a drawable circuit: %s"), inputPath.c_str());
        return false;
    }

    if (subcktArray.size() > 1)
    {
        *error = wxT("Netlists containing multiple subcircuits are not supported yet.");
        return false;
    }

    svCircuit& circuit = subcktArray[0];
    if (circuit.getDevices().empty())
    {
        *error = wxT("The netlist circuit does not contain any supported devices.");
        return false;
    }

    circuit.placeDevices(SVPA_PLACE_NON_OVERLAPPED);

    const wxRect bounds = circuit.getBoundingBox();

    // Device bounds deliberately stay cheap to compute, while wires, junction
    // trunks, rail markers and horizontal annotations can extend past them.
    // Give the renderer a generous safety border here, then let the pixel crop
    // below remove the unused white area from the exported PNG.
    const int width = (bounds.x + bounds.width + 10) * DEFAULT_GRID_SIZE;
    const int height = (bounds.y + bounds.height + 10) * DEFAULT_GRID_SIZE;
    if (width <= 0 || height <= 0)
    {
        *error = wxT("Automatic placement produced an invalid image size.");
        return false;
    }

    wxBitmap bitmap(width, height, 32);
    if (!bitmap.IsOk())
    {
        *error = wxString::Format(
            wxT("Could not allocate an image of %d x %d pixels."), width, height);
        return false;
    }

    wxMemoryDC dc;
    dc.SelectObject(bitmap);
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.Clear();

    wxGraphicsContext* gc = wxGraphicsContext::Create(dc);
    if (!gc)
    {
        dc.SelectObject(wxNullBitmap);
        *error = wxT("Could not create an off-screen graphics context.");
        return false;
    }

    svDeviceFactory::initGraphics(gc, DEFAULT_GRID_SIZE);
    circuit.draw(gc, DEFAULT_GRID_SIZE);
    delete gc;

    dc.SelectObject(wxNullBitmap);

    wxImage image = bitmap.ConvertToImage();
    if (!image.IsOk())
    {
        *error = wxT("Could not convert the rendered schematic to an image.");
        return false;
    }

    // Tighten headless output around the actual schematic. Placement uses
    // generous virtual margins so routing and labels never clip, but exported
    // figures should not carry a large empty canvas into reports.
    const unsigned char* pixels = image.GetData();
    if (pixels)
    {
        int minX = image.GetWidth();
        int minY = image.GetHeight();
        int maxX = -1;
        int maxY = -1;

        for (int y=0; y<image.GetHeight(); y++)
        {
            for (int x=0; x<image.GetWidth(); x++)
            {
                const int idx = (y*image.GetWidth() + x) * 3;
                if (pixels[idx] < 245 ||
                    pixels[idx+1] < 245 ||
                    pixels[idx+2] < 245)
                {
                    minX = std::min(minX, x);
                    minY = std::min(minY, y);
                    maxX = std::max(maxX, x);
                    maxY = std::max(maxY, y);
                }
            }
        }

        if (maxX >= minX && maxY >= minY)
        {
            const int padding = 24;
            minX = std::max(0, minX-padding);
            minY = std::max(0, minY-padding);
            maxX = std::min(image.GetWidth()-1, maxX+padding);
            maxY = std::min(image.GetHeight()-1, maxY+padding);
            image = image.GetSubImage(
                wxRect(minX, minY, maxX-minX+1, maxY-minY+1));
        }
    }

    if (!image.SaveFile(outputPath, wxBITMAP_TYPE_PNG))
    {
        *error = wxString::Format(wxT("Could not save PNG image: %s"), outputPath.c_str());
        return false;
    }

    return true;
}

// IDs for the controls and the menu commands
enum
{
    SpiceViewer_ShowGrid = wxID_HIGHEST+1,
    SpiceViewer_OpenNVS,
    SpiceViewer_Export,
    SpiceViewer_OpenNetlist = wxID_OPEN,
    SpiceViewer_Quit = wxID_EXIT,

    SpiceViewer_Help = wxID_HELP,
    SpiceViewer_About = wxID_ABOUT
};

// ----------------------------------------------------------------------------
// resources
// ----------------------------------------------------------------------------

#include "icon.xpm"
#include "icon_small.xpm"


// ----------------------------------------------------------------------------
// private classes
// ----------------------------------------------------------------------------

// main application class
class SpiceViewerApp : public wxApp
{
public:
    SpiceViewerApp()
        : m_headless(false),
          m_devicesRegistered(false),
          m_headlessExitCode(0)
    {
    }

    virtual bool OnInit();
    virtual int OnRun();
    virtual int OnExit();

private:
    bool m_headless;
    bool m_devicesRegistered;
    int m_headlessExitCode;
};

// define a scrollable canvas for displaying the schematic
class SpiceViewerCanvas: public wxScrolledCanvas
{
public:
    SpiceViewerCanvas(wxFrame *parent);

    void OnPaint(wxPaintEvent &event);
    void OnMouseMove(wxMouseEvent &event);
    void OnMouseDown(wxMouseEvent &event);
    void OnMouseUp(wxMouseEvent &event);
    void OnMouseWheel(wxMouseEvent &event);

    void SetCircuit(const svCircuit& ckt)
    { 
        m_ckt = ckt; 
        UpdateVirtualSize();
        UpdateGraphics();
    }

    const svCircuit& GetCircuit() const
        { return m_ckt; }

    //! Updates all graphic objects cached in the current circuit (sub)objects.
    //! This function needs to be called only on new circuit (see SetCircuit())
    //! and in case the grid size has been changed (see OnMouseWheel()).
    void UpdateGraphics()
    {
        wxGraphicsContext *gc = wxGraphicsContext::Create(this);
        if (!gc)
            return;

        svDeviceFactory::initGraphics(gc, m_gridSize);
        delete gc;
    }

    void UpdateVirtualSize()
    {
        wxRect rc = m_ckt.getBoundingBox();
        SetVirtualSize((rc.x+rc.width+10)*m_gridSize,
                       (rc.y+rc.height+10)*m_gridSize);
    }

    void ShowGrid(bool b)
    {
        m_bShowGrid = b; 
        Refresh();
    }

private:        // misc vars
    svCircuit m_ckt;
    unsigned int m_gridSize;
    wxPen m_gridPen;
    bool m_bShowGrid;

private:        // vars for dragging
    svBaseDevice* m_pDraggedDev;
    wxPoint m_ptDraggedDevOffset; // in pixel coords
    int m_idxDraggedDev;

    wxDECLARE_EVENT_TABLE();
};

// main application frame
class SpiceViewerFrame : public wxFrame
{
public:
    SpiceViewerFrame(const wxString& title);

    // event handlers (these functions should _not_ be virtual)
    void OnShowGrid(wxCommandEvent& event);
    void OnOpenNetlist(wxCommandEvent& event);
    void OnOpenNVS(wxCommandEvent& event);
    void OnExportNVS(wxCommandEvent& event);
    void OnQuit(wxCommandEvent& event);

    void OnHelp(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

private:
    SpiceViewerCanvas* m_canvas;

    wxDECLARE_EVENT_TABLE();
};

// ============================================================================
// implementation
// ============================================================================

// ----------------------------------------------------------------------------
// SpiceViewerApp - the application class
// ----------------------------------------------------------------------------

wxIMPLEMENT_APP(SpiceViewerApp);

bool SpiceViewerApp::OnInit()
{
    svHeadlessOptions headlessOptions;
    const wxString programName = wxFileName(argv[0]).GetFullName();
    if (HasHeadlessCommandLineArguments(argc, argv))
    {
        m_headless = true;

        wxString error;
        if (!ParseHeadlessCommandLine(argc, argv, &headlessOptions, &error))
        {
            wxFprintf(stderr, wxT("Error: %s\n\n"), error.c_str());
            PrintCommandLineUsage(stderr, programName);
            m_headlessExitCode = 2;
            return true;
        }

        if (headlessOptions.showHelp)
        {
            PrintCommandLineUsage(stdout, programName);
            return true;
        }
    }
    else
    {
        // Call the base class initialization method for normal GUI startup.
        if (!wxApp::OnInit())
            return false;
    }

    svDeviceFactory::registerAllDevices();
    m_devicesRegistered = true;
    setlocale(LC_NUMERIC, "C");

#define SELF_TESTS 1
#if SELF_TESTS
    struct {
        const char* testString;
        double value;
    } test[] = 
    {
        { "1", 1 },
        { "2.3", 2.3 },
        { "2.3e-9", 2.3e-9 },
        { "23.3n", 23.3e-9 },
        { "2.3nF", 2.3e-9 },
        { "99.9pFaraD", 99.9e-12 },
        { "10V", 10 }
    };

    const double EPSILON = 1e-9;

    for (size_t i=0; i<WXSIZEOF(test); i++)
    {
        double temp;

        svString teststr(test[i].testString);

        wxASSERT(teststr.getValue(&temp));
        wxASSERT(fabs(temp - test[i].value) < EPSILON);
    }
#endif

    if (m_headless)
    {
        wxLogStderr stderrLog;
        svScopedLogTarget logTarget(&stderrLog);

        wxInitAllImageHandlers();

        wxString error;
        const bool rendered =
            RenderNetlistToPng(headlessOptions.inputPath, headlessOptions.outputPath, &error);

        if (!rendered)
        {
            wxFprintf(stderr, wxT("Error: %s\n"), error.c_str());
            m_headlessExitCode = 1;
            return true;
        }

        wxFprintf(stdout, wxT("Rendered %s to %s\n"),
                  headlessOptions.inputPath.c_str(),
                  headlessOptions.outputPath.c_str());
        return true;
    }

    // create the main application window
    SpiceViewerFrame *frame = new SpiceViewerFrame("Netlist viewer");

    // and show it (the frames, unlike simple controls, are not shown when
    // created initially)
    frame->Show(true);

    // success: wxApp::OnRun() will be called which will enter the main message
    // loop and the application will run. If we returned false here, the
    // application would exit immediately.
    return true;
}

int SpiceViewerApp::OnRun()
{
    if (m_headless)
        return m_headlessExitCode;

    return wxApp::OnRun();
}

int SpiceViewerApp::OnExit()
{
    if (m_devicesRegistered)
    {
        svDeviceFactory::unregisterAllDevices();
        svDeviceFactory::releaseGraphics();
    }

    return wxApp::OnExit();
}

// ----------------------------------------------------------------------------
// SpiceViewerFrame - main frame
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(SpiceViewerFrame, wxFrame)
    EVT_MENU(SpiceViewer_ShowGrid,    SpiceViewerFrame::OnShowGrid)
    EVT_MENU(SpiceViewer_OpenNetlist, SpiceViewerFrame::OnOpenNetlist)
    EVT_MENU(SpiceViewer_OpenNVS,     SpiceViewerFrame::OnOpenNVS)
    EVT_MENU(SpiceViewer_Export,      SpiceViewerFrame::OnExportNVS)
    EVT_MENU(SpiceViewer_Quit,        SpiceViewerFrame::OnQuit)

    EVT_MENU(SpiceViewer_Help,        SpiceViewerFrame::OnHelp)
    EVT_MENU(SpiceViewer_About,       SpiceViewerFrame::OnAbout)
wxEND_EVENT_TABLE()

SpiceViewerFrame::SpiceViewerFrame(const wxString& title)
                            : wxFrame(NULL, wxID_ANY, title)
{
    wxIconBundle bundle;
    bundle.AddIcon(wxIcon(icon_xpm));
    bundle.AddIcon(wxIcon(icon_small_xpm));
    SetIcons(bundle);

    wxMenu *fileMenu = new wxMenu;
    fileMenu->AppendCheckItem(SpiceViewer_ShowGrid, "&Show grid", 
                              "Should the grid for the devices be shown?")->Check();
    fileMenu->Append(SpiceViewer_OpenNetlist, "&Open SPICE netlist...", "Open a SPICE netlist to view");
    fileMenu->Append(SpiceViewer_OpenNVS, "Open NVS...", "Open a schematic in the native NetlistViewer format (NVS)");
    fileMenu->AppendSeparator();
    fileMenu->Append(SpiceViewer_Export, "Export to NVS...", "Export the schematic to a native NetlistViewer format (NVS)");
    fileMenu->AppendSeparator();
    fileMenu->Append(SpiceViewer_Quit, "E&xit\tAlt-X", "Quit this program");

        // TODO: export routine for gEDA: http://geda.seul.org/wiki/geda:file_format_spec

    wxMenu *helpMenu = new wxMenu;
    helpMenu->Append(SpiceViewer_Help, "&Help...\tF1", "Show help page");
    helpMenu->Append(SpiceViewer_About, "&About...", "Show about dialog");

    // now append the freshly created menu to the menu bar...
    wxMenuBar *menuBar = new wxMenuBar();
    menuBar->Append(fileMenu, "&File");
    menuBar->Append(helpMenu, "&Help");

    // ... and attach this menu bar to the frame
    SetMenuBar(menuBar);

    // create a status bar just for fun (by default with 1 pane only)
    CreateStatusBar(1);
    SetStatusText("Welcome to Netlist Viewer " SW_VERSION_STR "!");

    // create the main canvas of this application
    // (it will get automatically linked to this frame and its size will be
    //  adjusted to fill the entire window)
    m_canvas = new SpiceViewerCanvas(this);
}

void SpiceViewerFrame::OnShowGrid(wxCommandEvent& event)
{
    if (m_canvas)
        m_canvas->ShowGrid(event.IsChecked());
}

void SpiceViewerFrame::OnOpenNetlist(wxCommandEvent& WXUNUSED(event))
{
    wxString defaultPath = wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath();
    wxFileDialog 
        openFileDialog(this, "Open SPICE netlist", defaultPath, "",
                       FILTER_SPICENETLIST, wxFD_OPEN|wxFD_FILE_MUST_EXIST);

    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...
    
    // proceed loading the file chosen by the user:
    svParserSPICE parser;
    svCircuitArray subcktArray; 
    if (!parser.load(subcktArray, openFileDialog.GetPath().ToStdString()))
    {
        wxLogError("Error while parsing the netlist file '%s'", openFileDialog.GetPath());
        return;
    }

    if (subcktArray.size() == 0)
    {
        wxLogError("The nelist file '%s' didn't contain any subcircuit", openFileDialog.GetPath());
        return;
    }

    if (subcktArray.size() > 1)
    {
        wxLogError("Sorry: multi-subcircuits not supported yet...");
        return;
    }

    subcktArray[0].placeDevices(SVPA_PLACE_NON_OVERLAPPED);
    m_canvas->SetCircuit(subcktArray[0]);
    SetTitle(wxString::Format("Netlist Viewer [%s]", subcktArray[0].getName()));

    Refresh();
}

void SpiceViewerFrame::OnOpenNVS(wxCommandEvent& WXUNUSED(event))
{
    wxString defaultPath = wxFileName(wxStandardPaths::Get().GetExecutablePath()).GetPath();
    wxFileDialog 
        openFileDialog(this, "Open NetlistViewer schematic", defaultPath, "",
                       FILTER_NETLISTVIEWERSCHEMATIC, wxFD_OPEN|wxFD_FILE_MUST_EXIST);

    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...

    // proceed loading the file chosen by the user:
    std::ifstream ifs((const char*)openFileDialog.GetPath());
    if (!ifs.fail())
    {
        try {
            boost::archive::text_iarchive ia(ifs);
            svDeviceFactory::registerAllDevicesForSerialization(ia);

            // read class state from archive
            svCircuit ckt;
            ia >> ckt;
            m_canvas->SetCircuit(ckt);

            SetTitle(wxString::Format("Netlist Viewer [%s]", ckt.getName()));
            Refresh();
        } 
        catch (const boost::archive::archive_exception& e)
        {
            wxLogError("Error while importing the NVS file: %s", e.what());
        }
    }
    else
    {
        wxLogError("Error while trying to open the NVS file '%s'", openFileDialog.GetPath());
        return;
    }
}

void SpiceViewerFrame::OnExportNVS(wxCommandEvent& WXUNUSED(event))
{
    wxFileDialog 
        saveFileDialog(this, "Save NetlistViewer schematic", "", "",
                       "NetlistViewer schematic (*.nvs)|*.nvs", wxFD_SAVE|wxFD_OVERWRITE_PROMPT);

    if (saveFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...

    // save data to archive
    std::ofstream ofs((const char*)saveFileDialog.GetPath());
    if (!ofs.fail())
    {
        try {
            boost::archive::text_oarchive oa(ofs);
            svDeviceFactory::registerAllDevicesForSerialization(oa);

            // write class instance to archive
            oa << m_canvas->GetCircuit();
        }
        catch (const boost::archive::archive_exception& e)
        {
            // NOTE: this is typically a logic error in the program!
            wxLogError("Error while exporting in NVS format: %s", e.what());
        }
    }
    else
    {
        wxLogError("Error while saving the NVS file '%s'", saveFileDialog.GetPath());
        return;
    }
}

void SpiceViewerFrame::OnQuit(wxCommandEvent& WXUNUSED(event))
{
    Close(true /* force the frame to close */);
}

void SpiceViewerFrame::OnHelp(wxCommandEvent& WXUNUSED(event))
{
    if (!wxLaunchDefaultBrowser(HELP_PAGE))
        wxLogError("Could not open the URL '%s'... please open it manually.", HELP_PAGE);
}

void SpiceViewerFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    wxAboutDialogInfo aboutInfo;
    aboutInfo.SetName("Netlist Viewer");
    aboutInfo.SetVersion(SW_VERSION_STR);
    aboutInfo.SetDescription("SPICE netlist viewer. This program converts a SPICE text netlist to a graphical schematic.");
    aboutInfo.SetCopyright(SW_COPYRIGHT_STR);
    aboutInfo.SetWebSite("https://github.com/f18m/netlist-viewer");
    aboutInfo.AddDeveloper("Francesco Montorsi <francesco.montorsi@gmail.com>");

    wxAboutBox(aboutInfo);
}

// ----------------------------------------------------------------------------
// SpiceViewerCanvas
// ----------------------------------------------------------------------------

wxBEGIN_EVENT_TABLE(SpiceViewerCanvas, wxScrolledCanvas)
    EVT_PAINT(SpiceViewerCanvas::OnPaint)

    EVT_MOUSEWHEEL(SpiceViewerCanvas::OnMouseWheel)
    EVT_MOTION(SpiceViewerCanvas::OnMouseMove)

    // left&right mouse buttons:
    EVT_LEFT_DOWN(SpiceViewerCanvas::OnMouseDown)
    EVT_LEFT_UP(SpiceViewerCanvas::OnMouseUp)
    EVT_RIGHT_UP(SpiceViewerCanvas::OnMouseUp)
wxEND_EVENT_TABLE()

SpiceViewerCanvas::SpiceViewerCanvas(wxFrame *parent)
        : wxScrolledCanvas(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                           wxHSCROLL | wxVSCROLL | wxFULL_REPAINT_ON_RESIZE)
{
    m_pDraggedDev = NULL;
    m_idxDraggedDev = wxNOT_FOUND;
    m_gridSize = DEFAULT_GRID_SIZE;
    m_gridPen = wxPen(*wxLIGHT_GREY, 1, wxPENSTYLE_DOT);
    m_bShowGrid = true;

    SetScrollRate(m_gridSize/10, m_gridSize/10);
    SetCursor(wxCURSOR_CROSS);
    SetBackgroundStyle(wxBG_STYLE_CUSTOM);
}

void SpiceViewerCanvas::OnPaint(wxPaintEvent &WXUNUSED(event))
{
#if 1
    wxPaintDC dc(this);
    DoPrepareDC(dc);
#else
    wxBufferedPaintDC dc(this, wxBUFFER_VIRTUAL_AREA);
#endif

    // clear our background
    dc.SetBackground(*wxWHITE_BRUSH);
    dc.SetBackgroundMode(wxSOLID);
    dc.SetPen(*wxTRANSPARENT_PEN);
    //dc.Clear();  -- doesn't clear all the virtual area!
    dc.DrawRectangle(wxPoint(0,0), GetVirtualSize()+wxSize(1,1));

    // draw the grid
    if (m_bShowGrid)
    {
        dc.SetPen(m_gridPen);
        wxSize sz = GetVirtualSize();
        for (int xx=m_gridSize; xx<sz.GetWidth(); xx+=m_gridSize)
            dc.DrawLine(xx, 0, xx, sz.GetHeight());
        for (int yy=m_gridSize; yy<sz.GetHeight(); yy+=m_gridSize)
            dc.DrawLine(0, yy, sz.GetWidth(), yy);
    }

    wxGraphicsContext *gc = wxGraphicsContext::Create(dc);
    if (!gc)
        return;

    // draw the schematic currently loaded
    m_ckt.draw(gc, m_gridSize, m_pDraggedDev ? m_idxDraggedDev : wxNOT_FOUND);
    delete gc;
}

void SpiceViewerCanvas::OnMouseDown(wxMouseEvent &event)
{
    if (m_pDraggedDev != NULL || !event.LeftDown())
        return;

    wxClientDC dc(this);
    DoPrepareDC(dc);

    wxPoint click(event.GetLogicalPosition(dc));
    int idx = m_ckt.hitTest(click, m_gridSize, m_gridSize/5 /* tolerance in px */);
    if (idx == wxNOT_FOUND)
        return;

    // the device we're dragging:
    m_pDraggedDev = m_ckt.getDevices().at(idx);
    m_idxDraggedDev = idx;

    // the offset (in pixel) between the clicked point and the reference node of the dragged device
    m_ptDraggedDevOffset = m_pDraggedDev->getGridPosition()*m_gridSize - click;

    Refresh();
}

void SpiceViewerCanvas::OnMouseMove(wxMouseEvent &event)
{
    if (!m_pDraggedDev)
        return;

    wxClientDC dc(this);
    DoPrepareDC(dc);
    wxPoint click(event.GetLogicalPosition(dc));

    // compute the delta of the distance (in grid units) from the original
    // dragged device's position
    double dx = double(click.x + m_ptDraggedDevOffset.x)/m_gridSize - m_pDraggedDev->getGridPosition().x;
    double dy = double(click.y + m_ptDraggedDevOffset.y)/m_gridSize - m_pDraggedDev->getGridPosition().y;

    // are we closer to another grid point?
    if (fabs(dx)>0.5 || fabs(dy)>0.5)
    {
        // get the position of the closest grid point
        wxPoint newGridPt = m_pDraggedDev->getGridPosition() + wxPoint(wxRound(dx),wxRound(dy));

        // update&refresh
        m_pDraggedDev->setGridPosition(newGridPt);
        m_ckt.updateBoundingBox();
        UpdateVirtualSize();
        Refresh();
    }
}

void SpiceViewerCanvas::OnMouseUp(wxMouseEvent &event)
{
    if (event.LeftUp())
    {
        m_pDraggedDev = NULL;
        Refresh();
    }
    else if (event.RightUp() && m_pDraggedDev)
    {
        // rotate the device being dragged
        m_pDraggedDev->rotateClockwise();
        m_ckt.updateBoundingBox();
        UpdateVirtualSize();
        Refresh();
    }
}

void SpiceViewerCanvas::OnMouseWheel(wxMouseEvent &event)
{
    if (!event.ControlDown())
    {
        wxPoint offset;
        if (event.GetWheelAxis() == 0)      // vertical
            offset = wxPoint(0, -2*event.GetWheelRotation()/event.GetWheelDelta());
        else if (event.GetWheelAxis() == 1)      // horizontal
            offset = wxPoint(-2*event.GetWheelRotation()/event.GetWheelDelta(), 0);

        Scroll(GetViewStart() + offset);

        return;
    }

    // zoom!
    m_gridSize += 2*event.GetWheelRotation()/event.GetWheelDelta();
    if (m_gridSize < 10)
        m_gridSize = 10;
    else if (m_gridSize > 150)
        m_gridSize = 150;

    UpdateGraphics();
    UpdateVirtualSize();
    Refresh();
}
