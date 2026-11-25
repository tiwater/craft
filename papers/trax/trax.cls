% trax.cls — Modern Single-Column Technical Report Class for TRAX
% Clean, original arXiv-ready layout compatible with both Tectonic (XeTeX) and pdfLaTeX.
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{trax}[2026/11/25 TRAX Modern Single-Column Preprint Class]

\LoadClass[10pt,letterpaper]{article}

% --- Encoding & Typography (pdfLaTeX + Tectonic compatible) ---
\RequirePackage[T1]{fontenc}
\RequirePackage[utf8]{inputenc}
\RequirePackage{mathpazo}
\RequirePackage[scaled=0.92]{helvet}
\RequirePackage[scaled=0.86]{inconsolata}
\RequirePackage{microtype}
\linespread{1.14}

% --- Page Geometry ---
\RequirePackage[
  letterpaper,
  top=0.88in,
  bottom=0.88in,
  left=0.92in,
  right=0.92in,
  headheight=16pt,
  headsep=18pt,
  footskip=24pt
]{geometry}

% --- Core Packages ---
\RequirePackage[table]{xcolor}
\RequirePackage{amsmath,amssymb,amsfonts}
\RequirePackage{booktabs,multirow,tabularx,array}
\RequirePackage{graphicx}
\RequirePackage{enumitem}
\RequirePackage{titlesec}
\RequirePackage{fancyhdr}
\RequirePackage{caption}
\RequirePackage{tikz}
\usetikzlibrary{arrows.meta,positioning,fit,backgrounds,calc}
\RequirePackage[numbers,sort&compress]{natbib}
\RequirePackage{url}
\RequirePackage[
  colorlinks=true,
  linkcolor=traxblue,
  citecolor=traxblue,
  urlcolor=traxblue,
  pdfborder={0 0 0}
]{hyperref}

% --- Brand Color Palette ---
\definecolor{traxnavy}{HTML}{0F172A}
\definecolor{traxblue}{HTML}{1D4ED8}
\definecolor{traxslate}{HTML}{334155}
\definecolor{traxmuted}{HTML}{64748B}
\definecolor{traxbg}{HTML}{F8FAFC}
\definecolor{traxtint}{HTML}{EFF6FF}
\definecolor{traxborder}{HTML}{CBD5E1}
\definecolor{traxgreen}{HTML}{15803D}
\definecolor{traxgreenbg}{HTML}{F0FDF4}
\definecolor{traxamber}{HTML}{B45309}
\definecolor{traxamberbg}{HTML}{FFFBEB}

% --- Paragraph & List Spacing ---
\setlength{\parindent}{0pt}
\setlength{\parskip}{5pt}
\setlength{\emergencystretch}{3em}
\tolerance=1500
\hbadness=2500
\let\trax@origunderscore\_
\renewcommand{\_}{\trax@origunderscore\allowbreak}
\setlist[itemize]{leftmargin=1.4em, itemsep=2.5pt, topsep=3pt, parsep=1pt}
\setlist[enumerate]{leftmargin=1.6em, itemsep=2.5pt, topsep=3pt, parsep=1pt}

% --- Float & Table Defaults ---
\setlength{\tabcolsep}{5.2pt}
\renewcommand{\arraystretch}{1.18}
\setcounter{topnumber}{4}
\setcounter{bottomnumber}{2}
\setcounter{totalnumber}{6}
\renewcommand{\topfraction}{0.92}
\renewcommand{\bottomfraction}{0.80}
\renewcommand{\textfraction}{0.08}
\renewcommand{\floatpagefraction}{0.85}

% --- Captions ---
\captionsetup{
  font={small,sf},
  labelfont={bf,color=traxnavy},
  textfont={color=traxslate},
  labelsep=period,
  skip=7pt
}
\captionsetup[table]{position=top, skip=6pt}

% --- Section Headings ---
\titleformat{\section}
  {\Large\sffamily\bfseries\color{traxnavy}}
  {\color{traxblue}\thesection}{0.65em}{}
\titlespacing*{\section}{0pt}{16pt plus 3pt minus 2pt}{6pt plus 1pt}

\titleformat{\subsection}
  {\normalsize\sffamily\bfseries\color{traxnavy}}
  {\color{traxslate}\thesubsection}{0.55em}{}
\titlespacing*{\subsection}{0pt}{12pt plus 2pt minus 1pt}{4pt plus 1pt}

\titleformat{\subsubsection}
  {\normalsize\sffamily\bfseries\color{traxslate}}
  {\thesubsubsection}{0.5em}{}
\titlespacing*{\subsubsection}{0pt}{8pt plus 1pt minus 1pt}{2pt}

% --- Running Headers & Footers ---
\newcommand{\@traxrunningtitle}{TRAX Technical Report}
\newcommand{\runningtitle}[1]{\renewcommand{\@traxrunningtitle}{#1}}

\fancypagestyle{traxmain}{
  \fancyhf{}
  \fancyhead[L]{\footnotesize\sffamily\color{traxmuted}\@traxrunningtitle}
  \fancyhead[R]{\footnotesize\sffamily\bfseries\color{traxslate}\thepage}
  \renewcommand{\headrulewidth}{0.5pt}
  \renewcommand{\headrule}{\hbox to\headwidth{\color{traxborder}\leaders\hrule height \headrulewidth\hfill}}
  \renewcommand{\footrulewidth}{0pt}
}

\fancypagestyle{traxfirst}{
  \fancyhf{}
  \fancyfoot[C]{\footnotesize\sffamily\color{traxmuted}\thepage}
  \renewcommand{\headrulewidth}{0pt}
  \renewcommand{\footrulewidth}{0pt}
}

\pagestyle{traxmain}

% --- Metadata Macros ---
\newcommand{\@traxsubtitle}{}
\newcommand{\@traxauthors}{}
\newcommand{\@traxaffil}{}
\newcommand{\@traxlinks}{}
\newcommand{\@traxdate}{\today}
\newcommand{\@traxabstract}{}
\newcommand{\@traxkeywords}{}
\newcommand{\@traxbadge}{Technical Report}

\newcommand{\traxbadge}[1]{\renewcommand{\@traxbadge}{#1}}
\newcommand{\traxauthors}[1]{\renewcommand{\@traxauthors}{#1}}
\newcommand{\traxaffil}[1]{\renewcommand{\@traxaffil}{#1}}
\newcommand{\traxlinks}[1]{\renewcommand{\@traxlinks}{#1}}
\newcommand{\traxdate}[1]{\renewcommand{\@traxdate}{#1}}
\newcommand{\traxabstract}[1]{\renewcommand{\@traxabstract}{#1}}
\newcommand{\traxkeywords}[1]{\renewcommand{\@traxkeywords}{#1}}

% --- Modern Single-Column Header & Abstract Card ---
\newcommand{\makeheader}{%
  \thispagestyle{traxfirst}%
  \vspace*{-18pt}%
  {\noindent\sffamily\footnotesize
    \textbf{\color{traxblue}TRAX}\;\textcolor{traxborder}{|}\;\textcolor{traxslate}{\@traxbadge}%
    \hfill\textcolor{traxmuted}{\@traxdate}%
  }\par
  \vspace{3pt}
  {\noindent\color{traxblue}\rule{\linewidth}{1.1pt}}\par
  \vspace{10pt}
  {\noindent\raggedright\fontsize{17.5}{21.5}\selectfont\sffamily\bfseries\color{traxnavy}\@title\par}
  \vspace{7pt}
  {\noindent\sffamily\normalsize\bfseries\color{traxnavy}\@traxauthors\par}
  \vspace{1.5pt}
  {\noindent\sffamily\small\color{traxslate}\@traxaffil\par}
  \vspace{4pt}
  {\noindent\sffamily\footnotesize\color{traxslate}\@traxlinks\par}
  \vspace{9pt}
  \noindent\begin{tikzpicture}
    \node[
      draw=traxborder,
      fill=traxbg,
      rounded corners=4pt,
      line width=0.6pt,
      inner xsep=12pt,
      inner ysep=8.5pt,
      text width=\dimexpr\linewidth-25.2pt\relax,
      align=justify
    ] (absbox) {%
      {\sffamily\small\bfseries\color{traxnavy}Abstract}\par\vspace{3pt}%
      {\linespread{1.05}\selectfont\footnotesize\color{traxnavy}\@traxabstract\par}%
      \ifx\@traxkeywords\@empty\else
        \vspace{5pt}%
        {\linespread{1.05}\selectfont\scriptsize\sffamily\color{traxslate}\textbf{\color{traxnavy}Keywords:} \@traxkeywords\par}%
      \fi
    };
    \fill[traxblue, rounded corners=1.5pt]
      ([xshift=0.3pt,yshift=-0.3pt]absbox.north west)
      rectangle
      ([xshift=3.2pt,yshift=0.3pt]absbox.south west);
  \end{tikzpicture}\par
  \vspace{6pt}
}

% --- Clean Callout Box ---
\newenvironment{traxcallout}[1][Note]{%
  \par\vspace{4pt}\noindent
  \begin{tikzpicture}
    \node[
      draw=traxborder,
      fill=traxbg,
      rounded corners=3pt,
      line width=0.5pt,
      inner xsep=10pt,
      inner ysep=7pt,
      text width=\dimexpr\linewidth-21pt\relax,
      align=justify
    ] (cbox) \bgroup
    {\sffamily\footnotesize\bfseries\color{traxblue}#1.}\hspace{0.4em}\small\color{traxslate}%
}{%
    \egroup;
  \end{tikzpicture}\par\vspace{4pt}%
}

\endinput
