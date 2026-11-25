% trax.cls — Clean Academic Single-Column Preprint Class for TRAX
% Classic serif academic paper layout (Palatino, centered title block, Page-1 teaser,
% unboxed centered Abstract, indented paragraphs, "Figure 1 |" captions).
% Compatible with both Tectonic (XeTeX) and arXiv pdfLaTeX.
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{trax}[2026/11/25 TRAX Academic Single-Column Preprint Class]

\LoadClass[11pt,a4paper]{article}

% --- Page Geometry (A4, generous classic academic margins) ---
\RequirePackage[
  a4paper,
  top=2.45cm,
  bottom=2.55cm,
  left=2.5cm,
  right=2.5cm,
  headheight=18pt,
  headsep=14pt,
  footskip=24pt
]{geometry}

% --- Typography (Classic Academic Palatino + subtle monospace) ---
\RequirePackage[T1]{fontenc}
\RequirePackage[utf8]{inputenc}
\RequirePackage[sc]{mathpazo}
\RequirePackage[scaled=0.90]{helvet}
\RequirePackage[scaled=0.84]{inconsolata}
\RequirePackage{microtype}
\linespread{1.08}

% --- Core Packages ---
\RequirePackage[table]{xcolor}
\RequirePackage{amsmath,amssymb,amsfonts}
\RequirePackage{booktabs,multirow,tabularx,array}
\RequirePackage{graphicx}
\RequirePackage{enumitem}
\RequirePackage{titlesec}
\RequirePackage{fancyhdr}
\RequirePackage{caption}
\RequirePackage{etoolbox}
\RequirePackage{tikz}
\usetikzlibrary{arrows.meta,positioning,fit,backgrounds,calc}
\RequirePackage[numbers,sort&compress]{natbib}
\RequirePackage{xurl}

% --- Tasteful Restrained Academic Palette ---
\definecolor{traxink}{HTML}{1A1F29}      % Deep charcoal ink
\definecolor{traxaccent}{HTML}{1B5E7A}   % Restrained academic slate-teal
\definecolor{traxmuted}{HTML}{5F6C7B}    % Metadata / secondary captions
\definecolor{traxrule}{HTML}{D0D7DE}     % Delicate hairline rules
\definecolor{traxbg}{HTML}{F7F9FB}       % Subtle diagram card fill
\definecolor{traxrow}{HTML}{F0F6FA}      % Whisper-light TRAX table row highlight
\definecolor{traxgreen}{HTML}{1E6B45}    % Muted verification green
\definecolor{traxgreenbg}{HTML}{F2FAF5}
\definecolor{traxamber}{HTML}{9A5B13}    % Muted caution amber
\definecolor{traxamberbg}{HTML}{FDF8F0}

\RequirePackage[
  colorlinks=true,
  linkcolor=traxaccent,
  citecolor=traxaccent,
  urlcolor=traxaccent,
  pdfborder={0 0 0}
]{hyperref}

% --- Classic Academic Paragraph Style (Indented, balanced spacing) ---
\setlength{\parindent}{1.5em}
\setlength{\parskip}{3pt plus 1pt}
\setlength{\emergencystretch}{2.5em}
\tolerance=1400
\hbadness=2500
\let\trax@origunderscore\_
\renewcommand{\_}{\trax@origunderscore\allowbreak}

\setlist[itemize]{leftmargin=1.6em, itemsep=2pt, topsep=3pt, parsep=0pt}
\setlist[enumerate]{leftmargin=1.7em, itemsep=2pt, topsep=3pt, parsep=0pt}

% --- Tables & Floats ---
\setlength{\tabcolsep}{5pt}
\renewcommand{\arraystretch}{1.15}
\AtBeginEnvironment{table}{\let\small\footnotesize\let\footnotesize\scriptsize}
\setcounter{topnumber}{4}
\setcounter{bottomnumber}{2}
\setcounter{totalnumber}{6}
\renewcommand{\topfraction}{0.92}
\renewcommand{\bottomfraction}{0.80}
\renewcommand{\textfraction}{0.08}
\renewcommand{\floatpagefraction}{0.85}

% --- Classic "Figure 1 | ..." & "Table 1 | ..." Captions ---
\DeclareCaptionLabelSeparator{traxbar}{\;\textcolor{traxaccent}{\textbf{|}}\;}
\captionsetup{
  font=small,
  labelfont={bf,color=traxink},
  textfont={color=traxink},
  labelsep=traxbar,
  skip=7pt
}
\captionsetup[table]{position=top, skip=6pt}

% --- Classic Academic Serif Section Headings ---
\titleformat{\section}
  {\Large\bfseries\color{traxink}}
  {\thesection}{0.75em}{}
\titlespacing*{\section}{0pt}{16pt plus 3pt minus 2pt}{6pt plus 1pt}

\titleformat{\subsection}
  {\large\bfseries\color{traxink}}
  {\thesubsection}{0.65em}{}
\titlespacing*{\subsection}{0pt}{12pt plus 2pt minus 1pt}{4pt plus 1pt}

\titleformat{\subsubsection}
  {\normalsize\bfseries\color{traxink}}
  {\thesubsubsection}{0.55em}{}
\titlespacing*{\subsubsection}{0pt}{8pt plus 1pt minus 1pt}{2pt}

% --- Delicate Academic Headers & Footers ---
\newcommand{\@traxrunningtitle}{TRAX: Transactional, Crash-Recoverable, and Ownership-Separated Code Policies}
\newcommand{\runningtitle}[1]{\renewcommand{\@traxrunningtitle}{#1}}
\newcommand{\@traxheaderleft}{Tiwater}
\newcommand{\traxheaderleft}[1]{\renewcommand{\@traxheaderleft}{#1}}
\newcommand{\@traxdate}{November 2026}
\newcommand{\traxdate}[1]{\renewcommand{\@traxdate}{#1}}

\fancypagestyle{traxmain}{
  \fancyhf{}
  \fancyhead[L]{\footnotesize\color{traxmuted}\@traxrunningtitle}
  \fancyhead[R]{\footnotesize\color{traxmuted}\@traxdate}
  \fancyfoot[C]{\small\color{traxink}\thepage}
  \renewcommand{\headrulewidth}{0.4pt}
  \renewcommand{\headrule}{\hbox to\headwidth{\color{traxrule}\leaders\hrule height \headrulewidth\hfill}}
  \renewcommand{\footrulewidth}{0pt}
}

\fancypagestyle{traxfirst}{
  \fancyhf{}
  \fancyhead[L]{\footnotesize\sffamily\bfseries\color{traxink}TRAX\;\normalfont\color{traxrule}|\;\color{traxmuted}\@traxheaderleft}
  \fancyhead[R]{\footnotesize\sffamily\color{traxmuted}\@traxdate}
  \fancyfoot[C]{\small\color{traxink}\thepage}
  \renewcommand{\headrulewidth}{0.45pt}
  \renewcommand{\headrule}{\hbox to\headwidth{\color{traxrule}\leaders\hrule height \headrulewidth\hfill}}
  \renewcommand{\footrulewidth}{0pt}
}

\pagestyle{traxmain}

% --- Metadata Macros ---
\newcommand{\@traxauthors}{}
\newcommand{\@traxaffil}{}
\newcommand{\@traxlinks}{}
\newcommand{\traxauthors}[1]{\renewcommand{\@traxauthors}{#1}}
\newcommand{\traxaffil}[1]{\renewcommand{\@traxaffil}{#1}}
\newcommand{\traxlinks}[1]{\renewcommand{\@traxlinks}{#1}}

% --- Authentic Centered Academic Title Block (DeepSeek-R1 / RAX style) ---
\newcommand{\makeacademictitle}{%
  \thispagestyle{traxfirst}%
  \begin{center}
    \vspace*{2pt}
    {\fontsize{15.5}{20.5}\selectfont\bfseries\color{traxink}\@title\par}
    \vspace{10pt}
    {\normalsize\bfseries\color{traxink}\@traxauthors\par}
    \vspace{3pt}
    {\small\color{traxmuted}\@traxaffil\par}
    \vspace{5pt}
    {\small\sffamily\color{traxmuted}\@traxlinks\par}
  \end{center}
  \vspace{8pt}
}

% --- Clean Centered Unboxed Academic Abstract ---
\renewenvironment{abstract}{%
  \vspace{6pt}
  \begin{center}
    {\large\bfseries\color{traxink}Abstract}
  \end{center}
  \vspace{-4pt}
  \noindent\ignorespaces
}{%
  \par\vspace{6pt}
}

\endinput
