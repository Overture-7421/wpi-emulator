#include "wizard/wizard_dialog.h"

#include <QHBoxLayout>
#include <QScrollArea>
#include <QVBoxLayout>

#include "wizard/project_page.h"
#include "wizard/robot_page.h"
#include "wizard/summary_page.h"
#include "wizard/toolchain_page.h"

namespace wpiemu::app {

WizardDialog::WizardDialog(AppServices& services, const UiResources& ui, QWidget* parent)
    : QDialog(parent),
      services_(services),
      ui_(ui),
      model_(services),
      titleLabel_(makeLabel({}, "title")),
      subtitleLabel_(makeLabel({}, "subtitle", nullptr, true)),
      stack_(new QStackedWidget),
      errorBanner_(new Banner(ui)),
      cancelButton_(new QPushButton(tr("Cancel"))),
      backButton_(new QPushButton(tr("Back"))),
      nextButton_(new QPushButton(tr("Next"))) {
    setWindowTitle(tr("Set up your robot"));
    setModal(true);
    resize(1040, 720);
    setMinimumSize(880, 620);
    setSizeGripEnabled(true);

    projectPage_ = new ProjectPage(model_, services_.recentProjects(), ui_);
    toolchainPage_ = new ToolchainPage(model_, ui_);
    robotPage_ = new RobotPage(model_, ui_);
    summaryPage_ = new SummaryPage(model_, ui_);
    for (WizardPage* page : std::initializer_list<WizardPage*>{projectPage_, toolchainPage_, robotPage_, summaryPage_}) {
        // Pages of different heights share the stack without resizing the dialog.
        auto* host = new QWidget;
        auto* l = new QVBoxLayout(host);
        l->setContentsMargins(0, 0, 0, 0);
        l->addWidget(page);
        if (page == projectPage_ || page == toolchainPage_ || page == summaryPage_) {
            auto* scroll = new QScrollArea;
            scroll->setWidgetResizable(true);
            scroll->setFrameShape(QFrame::NoFrame);
            scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
            scroll->setWidget(host);
            stack_->addWidget(scroll);
        } else {
            stack_->addWidget(host);
        }
    }

    // ---- sidebar ----
    auto* sidebar = new QWidget;
    sidebar->setObjectName(QStringLiteral("WizardSidebar"));
    sidebar->setFixedWidth(250);
    auto* sideLayout = new QVBoxLayout(sidebar);
    sideLayout->setContentsMargins(18, 24, 18, 18);
    sideLayout->setSpacing(6);
    auto* brand = new QHBoxLayout;
    auto* logo = new QLabel;
    logo->setPixmap(ui_.logo().pixmap(32, 32));
    brand->addWidget(logo);
    brand->addWidget(makeLabel(tr("WPI Emulator"), "section"));
    brand->addStretch(1);
    sideLayout->addLayout(brand);
    sideLayout->addSpacing(18);
    const QString stepNames[kWizardStepCount] = {tr("Project"), tr("Toolchain"), tr("Robot"), tr("Summary")};
    for (int i = 0; i < kWizardStepCount; ++i) {
        auto* b = new QPushButton(QStringLiteral("%1  %2").arg(i + 1).arg(stepNames[i]));
        b->setFlat(true);
        b->setProperty("stepButton", true);
        b->setProperty("current", false);
        b->setCursor(Qt::PointingHandCursor);
        b->setFocusPolicy(Qt::TabFocus);
        stepButtons_[static_cast<std::size_t>(i)] = b;
        sideLayout->addWidget(b);
        connect(b, &QPushButton::clicked, this, [this, i] { model_.goTo(static_cast<WizardStep>(i)); });
    }
    sideLayout->addStretch(1);
    sideLayout->addWidget(makeLabel(tr("Nothing is changed until you open the editor."), "muted", nullptr, true));

    // ---- content ----
    auto* content = new QWidget;
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(32, 28, 32, 18);
    contentLayout->setSpacing(6);
    contentLayout->addWidget(titleLabel_);
    contentLayout->addWidget(subtitleLabel_);
    contentLayout->addSpacing(10);
    contentLayout->addWidget(stack_, 1);
    contentLayout->addWidget(errorBanner_);

    auto* footer = new QWidget;
    footer->setObjectName(QStringLiteral("WizardFooter"));
    auto* footerLayout = new QHBoxLayout(footer);
    footerLayout->setContentsMargins(24, 12, 24, 12);
    footerLayout->setSpacing(10);
    footerLayout->addWidget(cancelButton_);
    footerLayout->addStretch(1);
    footerLayout->addWidget(backButton_);
    footerLayout->addWidget(nextButton_);
    nextButton_->setProperty("primary", true);
    nextButton_->setDefault(true);
    nextButton_->setMinimumWidth(130);

    auto* right = new QVBoxLayout;
    right->setSpacing(0);
    right->addWidget(content, 1);
    right->addWidget(footer);
    auto* root = new QHBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    root->addWidget(sidebar);
    root->addLayout(right, 1);

    connect(cancelButton_, &QPushButton::clicked, this, &QDialog::reject);
    connect(backButton_, &QPushButton::clicked, &model_, &WizardViewModel::back);
    connect(nextButton_, &QPushButton::clicked, this, &WizardDialog::onNext);
    connect(&model_, &WizardViewModel::stepChanged, this, &WizardDialog::onStepChanged);
    connect(&model_, &WizardViewModel::projectChanged, this, &WizardDialog::refreshChrome);
    connect(&model_, &WizardViewModel::toolchainChanged, this, &WizardDialog::refreshChrome);
    connect(&model_, &WizardViewModel::basicsChanged, this, &WizardDialog::refreshChrome);
    connect(&ui_, &UiResources::themeChanged, this, &WizardDialog::refreshStepIcons);

    onStepChanged();
}

WizardDialog::~WizardDialog() = default;

WizardPage* WizardDialog::currentPage() const {
    switch (model_.step()) {
        case WizardStep::Project: return projectPage_;
        case WizardStep::Toolchain: return toolchainPage_;
        case WizardStep::Robot: return robotPage_;
        case WizardStep::Summary: return summaryPage_;
    }
    return projectPage_;
}

void WizardDialog::setProjectPath(const QString& path) { model_.setProjectPath(path); }

void WizardDialog::onStepChanged() {
    stack_->setCurrentIndex(static_cast<int>(model_.step()));
    WizardPage* page = currentPage();
    titleLabel_->setText(page->title());
    subtitleLabel_->setText(page->subtitle());
    errorBanner_->clear();
    page->onEnter();
    if (QWidget* focus = page->initialFocus()) focus->setFocus();
    refreshChrome();
}

void WizardDialog::refreshStepIcons() {
    for (int i = 0; i < kWizardStepCount; ++i) {
        const auto step = static_cast<WizardStep>(i);
        QPushButton* b = stepButtons_[static_cast<std::size_t>(i)];
        const bool current = step == model_.step();
        const bool done = model_.stepComplete(step) && static_cast<int>(model_.step()) > i;
        b->setProperty("current", current);
        repolish(b);
        b->setIcon(done ? ui_.icon(QStringLiteral("check-circle"), IconTone::Success)
                        : ui_.icon(QStringLiteral("circle"), current ? IconTone::Accent : IconTone::Muted));
        b->setEnabled(i <= static_cast<int>(model_.furthestReachable()) || i <= static_cast<int>(model_.step()));
    }
}

void WizardDialog::refreshChrome() {
    refreshStepIcons();
    backButton_->setEnabled(model_.canGoBack());
    if (model_.isLastStep()) {
        nextButton_->setText(tr("Open editor"));
        nextButton_->setEnabled(model_.projectValid() && model_.basicsValid());
    } else {
        nextButton_->setText(tr("Next"));
        nextButton_->setEnabled(model_.canGoNext());
    }
    nextButton_->setIcon(model_.isLastStep() ? ui_.icon(QStringLiteral("edit-3d"), IconTone::OnAccent)
                                              : ui_.icon(QStringLiteral("arrow-right"), IconTone::OnAccent));
    nextButton_->setLayoutDirection(Qt::RightToLeft);   // icon after the text
    backButton_->setIcon(ui_.icon(QStringLiteral("arrow-left")));
}

void WizardDialog::onNext() {
    if (!model_.isLastStep()) {
        model_.next();
        return;
    }
    const WizardResult r = model_.finish();
    if (!r.ok) {
        errorBanner_->setMessage(BannerKind::Error, r.message);
        return;
    }
    result_ = r;
    accept();
}

}  // namespace wpiemu::app
