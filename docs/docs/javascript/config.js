window.MathJax = {
    tex: {
        inlineMath: [["\\(", "\\)"]["$", "$"]],
        displayMath: [["\\[", "\\]"], ["$$", "$$"]],
        processEscapes: true,
        processEnvironments: true,
    },
    extensions: ["AMSmath.js", "AMSsymbols.js"],
    options: {
        ignoreHtmlClass: ".*|",
        processHtmlClass: "arithmatex"
    }
};
