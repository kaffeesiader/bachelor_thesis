% UIBK Intelligent and Interactive Systems Thesis Class.
%
% # Based on the "book" class.
% # Uses A4 paper by default.
% # Uses 11pt type by default.
% # Sets titlepage layout, margins, etc.
% # Defines \chapternn command for non-numbered chapters.
% # Defines \declaration chapter + text.
%
% Authors:
% # 2012 Stephen Cobeldick
%
% justus[dot]piater[at]uibk[dot]ac[dot]at
%
% ### Options and Article Class ###
%
% These commands can be used in .tex:
\newcommand*{\uibkclass}{iisthesis}
\newcommand*{\uibkversion}{00.06}
\newcommand*{\uibkreleased}{2013/06/13}% LaTeX date format: yyyy/mm/dd
%
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{\uibkclass}[\uibkreleased\space\uibkversion\space UIBK IIS Thesis Class]
%
% TypeSize:
\newcommand*{\@uibktypesize}{11pt} % 11pt UIBK standard.
\DeclareOption{10pt}{\edef\@uibktypesize{\CurrentOption}}
\DeclareOption{11pt}{\edef\@uibktypesize{\CurrentOption}}
\DeclareOption{12pt}{\edef\@uibktypesize{\CurrentOption}}
%
% Columns:
\DeclareOption{twocolumn}{\OptionNotUsed{\CurrentOption}}
\DeclareOption{onecolumn}{\OptionNotUsed{\CurrentOption}}
%
% Thesis type:
\newif\ifbachelor\bachelorfalse
\newif\ifmaster\masterfalse
\DeclareOption{bachelor}{\bachelortrue}
\DeclareOption{master}{\mastertrue}
%
% All other options:
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
\ProcessOptions\relax
%
\LoadClass[a4paper,\@uibktypesize]{book}
%
% ### Packages ###
%
% !!! Don't forget the package list in the user guide !!!
%
% pdflatex:
\RequirePackage[scaled]{helvet}% Typeface.
% xelatex:
%\RequirePackage{fontspec}%      Type1 + TrueType + OpenType fonts in PDF.
%\RequirePackage[Scale=0.945]{Nimbus Sans L}% Typeface.
%
\RequirePackage{xcolor}%                Text + hyperlink colors.
\RequirePackage{natbib}%                Bibliography style.
%\RequirePackage{geometry}[2010/02/12]% Papersize + margins.
\RequirePackage{anysize}%	        Margins
\RequirePackage{graphicx}%              Insert logo image.
\RequirePackage{hyperref}[2007/04/09]%  Hyperlinks.
\RequirePackage[all]{hypcap}%           Hyperlinks to top of floats.
%
% ### UIBK Colors, Hyperref Defaults, Default Geometry ###
%
\definecolor{uibkblue}{cmyk}{1.0, 0.6, 0.0, 0.56}% {1.0, 0.6, 0.0, 0.56}
\definecolor{uibkorange}{cmyk}{0.0, 0.5, 1.0, 0.0}% {0.0, 0.5, 1.0, 0.0}
%
\newcommand*{\supervisor}[1]{
  \gdef\@supervisor{#1}%
}
\newcommand*{\@uibkauthor}{}
\renewcommand*{\author}[1]{% {name}
  \gdef\@author{#1}%
  \gdef\@uibkauthor{#1}%
  \hypersetup{pdfauthor=#1}%
}
\newcommand*{\@uibktitle}{}
\renewcommand{\title}[2][]{% [firstline]{secondline}
  \if&#1&%
    \gdef\@title{#2}%
    \gdef\@uibktitle{#2}%
    \hypersetup{pdftitle={#2}}
  \else%
    \gdef\@title{#1\\#2}%
    \gdef\@uibktitle{#1\\[2mm]#2}%
    \hypersetup{pdftitle={#1, #2}}
  \fi%
}
\@onlypreamble\title
%
%\geometry{
%  includehead,
%  headsep= 13mm,
%}
%
\hypersetup{
  citecolor       = uibkorange,
  citebordercolor = uibkorange,
  linkcolor       = uibkblue,
  linkbordercolor = uibkblue!40,
  urlcolor        = red,
  urlbordercolor  = red,
  pdfstartview    = {XYZ null null null}, % User default size.
  pdfpagemode     = UseOutlines,          % Open bookmarks.
}
\if@twoside
\else
  \hypersetup{colorlinks=true}
\fi%
%
% ### Bibliography Style, Autoref Names, Default Pagestyle ###
%
\setcitestyle{round}
\bibliographystyle{plainnat}
%
\renewcommand{\equationautorefname}{equation}
\renewcommand{\figureautorefname}{figure}
\renewcommand{\tableautorefname}{table}
%
%\let\familydefault\sfdefault
\normalfont
%
% Prevent compilation warning about repeated page numbers:
\pagenumbering{Alph}
%
% ### Table of Contents ###
%
\let\@uibktoc\tableofcontents
\def\tableofcontents{%
  \cleardoublepage%
  \phantomsection%
  \addcontentsline{toc}{chapter}{\contentsname}%
  \@uibktoc%
}%
\let\@uibktof\listoffigures
\def\listoffigures{%
  \cleardoublepage%
  \phantomsection%
  \addcontentsline{toc}{chapter}{\listfigurename}%
  \@uibktof%
}%
\let\@uibktot\listoftables
\def\listoftables{%
  \cleardoublepage%
  \phantomsection%
  \addcontentsline{toc}{chapter}{\listtablename}%
  \@uibktot%
}%
\let\@uibkbib\bibliography
\def\bibliography{%
  \cleardoublepage%
  \phantomsection%
  \addcontentsline{toc}{chapter}{\bibname}%
  \@uibkbib%
}
%
% ### No-Number Chapter ###
%
% \chapter*  | chapter title.
% -----------|------------------------------------------------------
% \chapternn | chapter title, bookmarks, headers, toc/contents line.
%
\newcommand*{\@uibkchap}[2][]{%
  \chapter*{#2}%
  \@mkboth{\MakeUppercase{#1}}{\MakeUppercase{#1}}%
  \addcontentsline{toc}{chapter}{#1}%
}
\newcommand*{\chapternn}{\@dblarg\@uibkchap}
%
% ### Declaration / Eid ###
%
\newcommand*{\uibkdeclaration}{%
  \def\declarationname{\ifgerman Erkl\"arung\else Declaration\fi}
  \chapter*{\declarationname}%
  \@mkboth{\MakeUppercase\declarationname}%
          {\MakeUppercase\declarationname}%
  \addcontentsline{toc}{chapter}{\declarationname}
  \@declaration
  \par\vspace{2cm}
  \noindent\ifgerman Signiert\else Signed\fi:\dotfill
  \ifgerman Datum\else Date\fi:\hbox to 4cm{\dotfill}
  % Main text body:
  \mainmatter%
  \pagestyle{headings}
}

\def\@declarationde{Ich erkl\"are hiermit an Eides statt durch meine eigenh\"andige Unterschrift, dass ich die vorliegende Arbeit selbst{\"a}ndig verfasst und keine anderen als die angegebenen Quellen und Hilfsmittel verwendet habe. Alle Stellen, die w{\"o}rtlich oder inhaltlich den angegebenen Quellen entnommen wurden, sind als solche kenntlich gemacht.}
\def\@declarationen{By my own signature I declare that I produced this work as the sole author, working independently, and that I did not use any sources and aids other than those referenced in the text.  All passages borrowed from external sources, verbatim or by content, are explicitly identified as such.}

%
% ### Draft Frame ###
%
\newcommand*{\@uibkframe}[3]{% {width}{height}{text}
  \hbox to #1{%
    \vrule height #2\hss%
    \vbox to #2{%
      \hrule width #1\vss%
      \rlap{\ttfamily\mdseries\upshape\fontsize{10}{10}\selectfont\,#3}%
      \vss\hrule%
    }%
    \hss\vrule%
  }%
}
%
% ### Title Page ###
%
% pdflatex:
\newcommand*{\@uibkLarge}{\sffamily\mdseries\upshape\fontsize{12}{12}\selectfont}
\newcommand*{\@uibkLARGE}{\sffamily\mdseries\upshape\fontsize{16}{16}\selectfont}
% xelatex:

%
\def\@uibklogofile{Uni_LOGO_4c.pdf}
\newcommand*{\logoimage}[1][]{\xdef\@uibklogofile{#1}}
\@onlypreamble\logoimage
%
\newcommand*{\ps@iisfront}{%
  % Sectionmarks:
  \let\@mkboth\@gobbletwo
  \let\sectionmark\@gobble
  \let\subsectionmark\@gobble
  % Header:
  \def\@oddhead{%
    \if&\@uibklogofile&
      %\@uibkframe{0.5\headheight}{\headheight}{UIBK}%
      \vspace{4mm}%
      \@uibkframe{3cm}{6cm}{UIBK}%
    \else%
      \vspace{4mm}%
      \includegraphics[height=6cm]{\@uibklogofile}%
    \fi%
    \hfill%
    \vbox to 6cm{%
      \vspace{4mm}%
      \llap{\@uibkLARGE\universityname}%
      \vfill
      \llap{\@uibkLarge\institutename}%
      \vspace{4mm}%
      \llap{\@uibkLarge\researchgroupname}%
    }%
  }%
  \let\@evenhead\@empty%
  % Footer:
  \let\@oddfoot\@empty%
  \let\@evenfoot\@empty%
}
%
\if@titlepage
  \renewcommand{\maketitle}{%
    \pagestyle{empty}% For ahead+rear of titlepage.
    \cleardoublepage%
    %\if@twoside
    %  \newgeometry{margin=30mm,includehead,head=57mm,headsep=0sp,left=35mm}% Allow for binding.
    %\else
    %  \newgeometry{margin=30mm,includehead,head=57mm,headsep=0sp}%
    %\fi%
    \thispagestyle{iisfront}% IIS titlepage
    {%
	  
      \let\footnotesize\small%
      \let\footnoterule\relax%
      \let \footnote \thanks%
      \raggedleft%
      \null%
      \vspace*{13cm}%
      %\vspace*{12cm}%
      {\@uibkLARGE\@uibktitle}%
     \\[20mm]%
     {\@uibkLarge\textbf{\@uibkauthor}}%
      \vfill\hfill
      {\@uibkLarge\textbf{\@thesis}}\\
      \medskip
      {\@uibkLarge\textbf{\@supervisortitle: }\@supervisor}\\
      {\@uibkLarge\@date}%
      \\
    }%
    %\restoregeometry%
    %default
    \if@twoside
      %\marginsize{left}{right}{top}{bottom}
       \marginsize{30mm}{20mm}{20mm}{20mm}
    \else
       \marginsize{25mm}{25mm}{20mm}{20mm}
    \fi%

    \frontmatter%
    \pagestyle{plain}% For front matter.
    \setcounter{footnote}{0}%
    \global\let\thanks\relax
    \global\let\maketitle\relax
    \global\let\@thanks\@empty
    \global\let\@author\@empty
    \global\let\@date\@empty
    \global\let\@title\@empty
    \global\let\title\relax
    \global\let\author\relax
    \global\let\date\relax
    \global\let\and\relax
  }
\fi
%
% ### Correct English Date Format ###
%
% Reset today's date:
\newcommand*{\settoday}[1]{\expandafter\@uibksettoday#1\@nil @@@}% {yyyy/mm/dd}
\def\@uibksettoday#1/#2/#3\@nil @@@{\year=#1\month=#2\day=#3}
\@onlypreamble\settoday
%
\newcommand*{\@uibktoday}{%
  \number\day%
  \ifcase\day%
    \or st\or nd\or rd\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or th\or st\or nd\or rd\or th\or th\or th\or th\or th\or th\or th\or st%
  \fi%
  \space%
  \ifcase\month%
    \or January\or February\or March\or April\or May\or June\or July\or August\or September\or October\or November\or December%
  \fi%
  \space%
  \number\year%
}
\let\today\@uibktoday
\def\dateuk{\let\today\@uibktoday}
\def\dateenglish{\let\today\@uibktoday}
\def\datebritish{\let\today\@uibktoday}
\def\dateUKenglish{\let\today\@uibktoday}
\def\dateaustralian{\let\today\@uibktoday}
\def\datenewzealand{\let\today\@uibktoday}
\AtBeginDocument{%
  \def\dateuk{\let\today\@uibktoday}
  \def\dateenglish{\let\today\@uibktoday}
  \def\datebritish{\let\today\@uibktoday}
  \def\dateUKenglish{\let\today\@uibktoday}
  \def\dateaustralian{\let\today\@uibktoday}
  \def\datenewzealand{\let\today\@uibktoday}
  \newif\ifgerman
  \@ifpackageloaded{babel}{}{\gdef\iflanguage#1#2#3{#3}}
  \iflanguage{ngerman}{\germantrue}{\germanfalse}
  \ifgerman
    \def\universityname{Universit\"at Innsbruck}
    \def\institutename{Institut f\"ur Informatik}
    \def\researchgroupname{Intelligente und Interaktive Systeme}
    \ifbachelor\def\@thesis{Bachelorarbeit}\fi
    \ifmaster\def\@thesis{Masterarbeit}\fi
    \def\@supervisortitle{Betreuer}
    \def\@declaration{\@declarationde}
  \else
    \def\universityname{University of Innsbruck}
    \def\institutename{Institute of Computer Science}
    \def\researchgroupname{Intelligent and Interactive Systems}
    \ifbachelor\def\@thesis{B.Sc.~Thesis}\fi
    \ifmaster\def\@thesis{M.Sc.~Thesis}\fi
    \def\@supervisortitle{Supervisor}
    \def\@declaration{\@declarationen}
  \fi
}
%
% End of iisthesis.cls
